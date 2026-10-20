"""
video.py - camera recording for CARLA episodes (visualisation only; never used for labels).

RecordingPlant is a CarlaPlant with a chase camera that follows the ego (behind and above, looking down the road, centred between the
two lanes): one RGB camera plus one instance-segmentation camera at the same pose.  Frames are taken from t0 only (the warm-up approach
and the blocker re-placement are not shown) and streamed straight into the MP4 (no clip is held in RAM).
Look: the generated site has no ground mesh or road texture, so each frame is composited - vehicle pixels (instance camera) are kept
as rendered, the empty void becomes grass, the road surface asphalt, the painted lines stay white.  Vehicles are painted by role
(ego blue, static obstacles red, moving actors orange); blueprint colour has no effect on the physics.
Partial observation (world "reveal" / "forget"): an actor the ego's observation feed does not deliver at time t is drawn as a faint
ghost labelled "not visible to ego"; the frame where it is delivered gets a banner.  Nothing physical hides it on CARLA.
install(): wraps harness.run_episode; the clip is: 1 s title card, the run in real time (dt 0.05 -> 20 fps), then - if the episode
ended at contact - the contact frame held with a CONTACT banner up to the horizon (no further simulation is shown: what CARLA does
after contact is not part of the evaluated run), then a 1 s result card.  G and termination come from the harness; video never changes them.

Rendering must be on: call enable_rendering(world) once after connect (plant_carla.configure_world turns it off).
"""
import math
import os
import queue

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .plant_carla import CarlaPlant

W, H, FPS = 1280, 720, 20
FONT = "C:/Windows/Fonts/segoeui.ttf"
FONT_B = "C:/Windows/Fonts/segoeuib.ttf"
RED, GREEN, AMBER = (235, 70, 70), (70, 210, 100), (255, 220, 80)
PAINT = {"ego": "0,90,255", "static": "200,30,30", "mover": "255,150,0"}
VEHICLE_TAGS = (12, 13, 14, 15, 16, 17, 18, 19)               # CARLA 0.9.16 semantic tags: pedestrian, rider, car .. bicycle


def _font(size, bold=False):
    try:
        return ImageFont.truetype(FONT_B if bold else FONT, size)
    except OSError:
        return ImageFont.load_default()


F_BIG, F_MID, F_SMALL = _font(46, True), _font(28, True), _font(24)


def enable_rendering(world, site=None, x_range=(-250.0, 450.0)):
    """Turn rendering on; with `site`, also paint the lane lines (the generated OpenDRIVE mesh has no markings): solid road edges,
    dashed divider between the ego (right) lane and the left lane.  Debug lines show in camera images; they stay for the session."""
    import carla
    s = world.get_settings()
    s.no_rendering_mode = False
    world.apply_settings(s)
    w = carla.WeatherParameters.ClearNoon                     # weather is visual only (no effect on the vehicle physics)
    w.fog_density, w.fog_distance, w.scattering_intensity, w.mie_scattering_scale = 0.0, 0.0, 0.0, 0.0   # haze washes the void out
    world.set_weather(w)
    if site is not None:
        cmap, w = world.get_map(), site.lane_w
        x0, x1 = x_range
        white = carla.Color(235, 235, 235)
        for lat, dash in ((0.5 * w, False), (-0.5 * w, True), (-1.5 * w, False)):
            x = x0
            while x < x1:
                seg = 3.0 if dash else 10.0
                a, b = site.to_world(x, lat, cmap, z_off=0.05), site.to_world(x + seg, lat, cmap, z_off=0.05)
                world.debug.draw_line(a, b, thickness=0.15, color=white, life_time=86400.0, persistent_lines=True)
                x += 9.0 if dash else seg
    world.tick()


def verdict(res, eps=0.0):
    if res.termination == "collision":
        return "FAILS: collision", RED
    if res.G is None or not math.isfinite(res.G):
        return "FAILS: margin -inf", RED
    if res.G >= eps:
        return f"SAFE: margin G = {res.G:+.3f}", GREEN
    return f"FAILS: margin G = {res.G:+.3f} (below 0)", RED


def _fit(d, text, font, maxw):
    """Shrink `font` (2 px at a time, floor 14) until `text` fits in maxw pixels; long case titles / policy names would run off the frame."""
    try:
        while d.textlength(text, font=font) > maxw and font.size > 14:
            font = font.font_variant(size=font.size - 2)
    except AttributeError:
        pass
    return font


def _box(d, xy, text, font, fill=(255, 255, 255), bg=(0, 0, 0, 170), pad=10):
    x, y = xy
    font = _fit(d, text, font, W - x - 2 * pad - 8)
    l, t, r, b = d.textbbox((x, y), text, font=font)
    d.rectangle((l - pad, t - pad, r + pad, b + pad), fill=bg)
    d.text((x, y), text, font=font, fill=fill)
    return b + pad


def composite(rgb, inst, ghost_ids=(), seed=0, tint=None):
    """Frame look (see module doc).  rgb (H, W, 3) uint8, inst (H, W, 4) BGRA instance image (R = semantic tag, G + 256 B = actor id).
    ghost_ids: actor ids drawn as faint ghosts (not delivered to the ego at this time).  tint: {actor id: (r, g, b)} role colour for
    vehicles whose blueprint has no paintable colour (shading kept from the render).  Returns (frame uint8, {id: bbox})."""
    A = rgb.astype(np.float32)
    tag = inst[..., 2]
    aid = inst[..., 1].astype(np.int32) + 256 * inst[..., 0].astype(np.int32)
    veh = np.isin(tag, VEHICLE_TAGS)
    r, b = A[..., 0], A[..., 2]
    void = ~veh & (b > r + 8)                                   # the empty void renders blue-grey; the road mesh warm grey
    line = ~veh & ~void & (A.mean(-1) > 215)
    road = ~veh & ~void & ~line
    h, w = tag.shape
    out = A.copy()
    rows = np.linspace(0.72, 1.05, h, dtype=np.float32)[:, None]   # darker toward the horizon for depth
    noise = np.random.default_rng(seed).normal(0, 6, (h, w)).astype(np.float32)
    grass = np.stack([62 * rows + noise, 118 * rows + noise, 52 * rows + noise], -1)
    out[void] = grass[void]
    asph = np.clip(A.mean(-1, keepdims=True) * 0.42 + np.array([4, 4, 8], np.float32), 0, 255)
    out[road] = np.broadcast_to(asph, A.shape)[road]
    out[line] = 245
    for i, col in (tint or {}).items():
        m = veh & (aid == i)
        if m.any():
            out[m] = np.clip(A[m].mean(-1, keepdims=True) / 110.0, 0.25, 1.6) * np.array(col, np.float32)
    boxes = {}
    for i in ghost_ids:
        m = veh & (aid == i)
        if m.any():
            ys, xs = np.nonzero(m)
            boxes[i] = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
            out[m] = 0.25 * out[m] + 0.75 * np.array([52, 52, 58], np.float32)     # ghost over asphalt
    return np.clip(out, 0, 255).astype(np.uint8), boxes


def _card(base, lines):
    im = Image.alpha_composite(Image.fromarray(base).convert("RGBA"), Image.new("RGBA", (base.shape[1], base.shape[0]), (0, 0, 0, 160)))
    d = ImageDraw.Draw(im)
    y = 180
    for text, font, col in lines:
        font = _fit(d, text, font, W - 160)
        d.text((80, y), text, font=font, fill=col)
        y += font.size + 24
    return np.asarray(im.convert("RGB"))


class _TickTake:
    """World proxy: tick() then immediately drain the camera image (see RecordingPlant._take); everything else is delegated."""

    def __init__(self, world, plant):
        self._w, self._p = world, plant

    def tick(self, *a):
        fid = self._w.tick(*a)
        if self._p.camera is not None:
            self._p._take(fid)
        return fid

    def __getattr__(self, name):
        return getattr(self._w, name)


class RecordingPlant(CarlaPlant):
    def __init__(self, *a, cam=(16.0, 9.0, -28.0, 75.0), **kw):
        super().__init__(*a, **kw)
        self.world = _TickTake(self.world, self)
        self._img = None
        self.cam_back, self.cam_up, self.cam_pitch, self.cam_fov = cam
        self.camera, self.q = None, queue.Queue()
        self.inst_cam, self.qi, self._inst = None, queue.Queue(), None
        self.vis = ({}, {})          # (reveal {actor: t}, forget {actor: t}) of the world being filmed, set by the wrapper
        self.frozen = None           # (frame, t, v, boxes) once the episode ended at contact
        self._prev_hidden, self._banner = None, None
        self._tint = {}
        self.clip = None             # (path, header, sub, note), set by the run_episode wrapper before build
        self.writer, self.n_frames, self.last, self.last_t = None, 0, None, 0.0

    def _cam_tf(self, ego_x):
        C = self.C
        loc = self.site.to_world(ego_x - self.cam_back, -0.5 * self.site.lane_w, self.map, z_off=self.cam_up)   # between the two lanes
        return C.Transform(loc, C.Rotation(pitch=self.cam_pitch, yaw=self.site.road_yaw))

    def _spawn(self, bp_id, x, lat, v, tag):
        """CarlaPlant._spawn with the blueprint painted by role (visual only)."""
        C = self.C
        bps = self.lib.filter(bp_id)
        bp = bps[0] if bps else [b for b in self.lib.filter("vehicle.*") if int(b.get_attribute("number_of_wheels")) == 4][0]
        bp.set_attribute("role_name", tag)
        role = PAINT["ego" if tag == "ego" else ("mover" if v else "static")]
        if bp.has_attribute("color") and bp.get_attribute("color").is_modifiable:
            bp.set_attribute("color", role)
            paint = None
        else:
            paint = tuple(int(c) for c in role.split(","))           # no paintable colour: tinted in the composite
        a = self.world.try_spawn_actor(bp, C.Transform(self._loc(x, lat), C.Rotation(yaw=self.site.road_yaw)))
        if a is None:
            raise RuntimeError(f"spawn failed for {tag} at x={x:.1f} lat={lat:.2f}")
        if v:
            a.set_target_velocity(C.Vector3D(self.site.f[0] * v, self.site.f[1] * v, 0.0))
        if paint:
            self._tint[a.id] = paint
        return a

    def _place(self, ego_x):
        tf = self._cam_tf(ego_x)
        for c in (self.camera, self.inst_cam):                    # both cameras share one pose (pixel-aligned masks)
            if c is not None:
                c.set_transform(tf)

    def _cam(self, kind, q, **attrs):
        bp = self.lib.find(kind)
        for k, v in {"image_size_x": str(W), "image_size_y": str(H), "fov": str(self.cam_fov), **attrs}.items():
            bp.set_attribute(k, v)
        c = self.world.spawn_actor(bp, self.C.Transform(self.site.origin.location, self.C.Rotation()))
        c.listen(q.put)
        return c

    def _spawn_camera(self):
        # auto-exposure on, bloom / lens flare off (with them the untextured site washes out to white)
        self.camera = self._cam("sensor.camera.rgb", self.q, exposure_mode="histogram", bloom_intensity="0.0", lens_flare_intensity="0.0")
        self.inst_cam = self._cam("sensor.camera.instance_segmentation", self.qi)

    def _take(self, frame_id):
        """Called by the world proxy straight after every tick.  The image MUST be drained before any other RPC: in synchronous mode a
        pending camera image blocks the server's game thread until the client takes it, so a getter (bounding_box, get_transform)
        issued first deadlocks the server (reproduced on 0.9.16)."""
        self._img = self._drain(self.q, frame_id)
        self._inst = self._drain(self.qi, frame_id) if self.inst_cam is not None else None
        if self._img is not None:
            self._img = self._img[:, :, 2::-1].copy()

    @staticmethod
    def _drain(q, frame_id):
        while True:
            try:
                im = q.get(timeout=20.0)
            except queue.Empty:
                return None
            if im.frame >= frame_id:
                return np.frombuffer(im.raw_data, dtype=np.uint8).reshape(im.height, im.width, 4).copy()

    def _hidden(self, t):
        """Names of actors the observation feed does not deliver at time t (queues.ObservationFeed visibility rule)."""
        reveal, forget = self.vis
        return [n for n in self.actors if n != "ego" and not (t >= reveal.get(n, -math.inf) and t < forget.get(n, math.inf))]

    def _emit(self, fr, t, v, boxes=None, banner=None):
        if self.writer is None:
            import imageio.v2 as imageio
            path, header, sub, note = self.clip
            os.makedirs(os.path.dirname(path), exist_ok=True)
            self.writer = imageio.get_writer(path, fps=FPS, codec="libx264", quality=8, macro_block_size=8)
            title = _card(fr, [(header, F_BIG, (255, 255, 255)), (sub, F_MID, (225, 225, 225))] +
                          ([(ln, F_SMALL, (210, 210, 210)) for ln in note.split("\n")] if note else []))
            for _ in range(FPS):
                self.writer.append_data(title)
        _, header, sub, _ = self.clip
        im = Image.fromarray(fr).convert("RGBA")
        lay = Image.new("RGBA", im.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(lay)
        for (x0, y0, x1, y1) in (boxes or {}).values():          # ghost: actor not delivered to the ego
            d.rectangle((x0 - 4, y0 - 4, x1 + 4, y1 + 4), outline=AMBER + (230,), width=3)
            ly = y0 - 44 if y0 > 150 else y1 + 14                 # below the box when the car is far up, under the header
            _box(d, (max(x0 - 40, 8), ly), "not visible to ego", F_SMALL, fill=AMBER)
        y = _box(d, (24, 20), header, F_MID)
        _box(d, (24, y + 8), sub, F_SMALL)
        _box(d, (24, H - 60), f"t = {t:4.2f} s    ego speed = {v:5.2f} m/s", F_SMALL)
        if banner:
            _box(d, (24, H - 120), banner[0], F_MID, fill=banner[1])
        self.writer.append_data(np.asarray(Image.alpha_composite(im, lay).convert("RGB")))
        self.last, self.last_t, self.n_frames = fr, t, self.n_frames + 1

    def build(self, spec):
        self.close()
        self._tint = {}
        if self.clip:
            self._spawn_camera()                                  # exists through warm-up; those frames are discarded
        self._building = True                                     # CarlaPlant.build calls close() first: keep the camera
        try:
            st = super().build(spec)
        finally:
            self._building = False
        if self.camera is not None:
            while not self.q.empty():
                self.q.get_nowait()
            self._place(st["ego"].x)
        return st

    def step(self, controls):
        snap = super().step(controls)
        if self.camera is not None:
            ego = snap.actors["ego"]
            self._place(ego.x)                                    # takes effect on the next tick
            if self._img is not None:
                hid = self._hidden(snap.t)
                fr, boxes = (composite(self._img, self._inst, [self.actors[n].id for n in hid], seed=self.n_frames, tint=self._tint)
                             if self._inst is not None else (self._img, {}))
                if self._prev_hidden is not None:
                    if any(n not in hid for n in self._prev_hidden):
                        self._banner = (f"revealed to ego at t = {snap.t:.2f} s", AMBER), 30
                    elif any(n not in self._prev_hidden for n in hid):
                        self._banner = (f"hidden again at t = {snap.t:.2f} s (ego must remember it)", AMBER), 30
                self._prev_hidden = hid
                banner = None
                if self._banner and self._banner[1] > 0:
                    banner, self._banner = self._banner[0], (self._banner[0], self._banner[1] - 1)
                if snap.events:
                    banner = (f"CONTACT at t = {snap.t:.2f} s - run ends here", RED)
                    self.frozen = (fr, snap.t, ego.v, boxes)
                self._emit(fr, snap.t, ego.v, boxes, banner)
        return snap

    def hold_contact(self, n_full):
        """After an episode that ended at contact: hold the contact frame (no more simulation) up to the horizon."""
        if self.frozen is None or self.writer is None:
            return
        fr, t, v, boxes = self.frozen
        while self.n_frames < n_full:
            self._emit(fr, t, v, boxes, (f"CONTACT at t = {t:.2f} s - run ends here", RED))

    def finish(self, res, eps=0.0):
        if self.writer is None:
            return False
        _, header, sub, _ = self.clip
        vtxt, vcol = verdict(res, eps)
        lines = [(header, F_MID, (255, 255, 255)), (sub, F_SMALL, (225, 225, 225)), (vtxt, F_BIG, vcol)]
        if res.termination == "collision":
            lines.append(("the run ends at first contact; what CARLA does after contact is not shown", F_SMALL, (210, 210, 210)))
        end = _card(self.last, lines)
        for _ in range(FPS):
            self.writer.append_data(end)
        self.writer.close()
        self.writer = None
        return True

    def close(self):
        if self.writer is not None:
            self.writer.close()
            self.writer = None
        if self.camera is not None and not getattr(self, "_building", False):
            for c in (self.camera, self.inst_cam):
                try:
                    c.stop()
                    c.destroy()
                except Exception:
                    pass
            self.camera, self.inst_cam = None, None
        super().close()


def install(out_dir, describe, eps=0.0):
    """Patch sim.harness.run_episode.  describe(spec, policy, world) -> (filename_stem, header, sub, note) or None (no video).
    Returns the list that collects one entry per written clip."""
    from . import harness
    orig = getattr(harness.run_episode, "_orig", harness.run_episode)
    written = []

    def run_episode(plant, spec, policy, world, *a, **kw):
        dsc = describe(spec, policy, world) if isinstance(plant, RecordingPlant) else None
        if dsc:
            stem, header, sub, note = dsc
            plant.clip, plant.n_frames = (os.path.join(out_dir, stem + ".mp4"), header, sub, note), 0
            plant.vis, plant.frozen = (world.get("reveal") or {}, world.get("forget") or {}), None
            plant._prev_hidden, plant._banner = None, None
        res = orig(plant, spec, policy, world, *a, **kw)
        if dsc:
            plant.hold_contact(int(round(spec["horizon_s"] / plant.dt)))
            if plant.finish(res, eps):
                written.append({"file": plant.clip[0], "policy": policy.name, "world": world.get("name"), "G": res.G,
                                "termination": res.termination, "verdict": verdict(res, eps)[0]})
            plant.clip = None
        return res

    run_episode._orig = orig
    harness.run_episode = run_episode
    return written
