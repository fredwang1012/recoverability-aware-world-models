"""
video.py - camera recording for CARLA episodes (visualisation only; never used for labels).

RecordingPlant is a CarlaPlant with one RGB camera that follows the ego (chase view: behind and above, looking down the road,
centred between the two lanes).  Frames are taken from t0 only (the warm-up approach and the blocker re-placement are not shown) and
streamed straight into the MP4 (no clip is held in RAM).
install(): wraps harness.run_episode so that, for a RecordingPlant, the episode is padded to the full horizon after an early stop
(collision) with the last controls held, and the clip is written as: 1 s title card, the run in real time (dt 0.05 -> 20 fps),
1 s result card.  The episode result (G, termination) is the one the harness computed; the padding never changes it.

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
RED, GREEN = (235, 70, 70), (70, 210, 100)


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
    world.set_weather(carla.WeatherParameters.ClearNoon)     # weather is visual only (no effect on the vehicle physics)
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


def _box(d, xy, text, font, fill=(255, 255, 255), bg=(0, 0, 0, 170), pad=10):
    x, y = xy
    l, t, r, b = d.textbbox((x, y), text, font=font)
    d.rectangle((l - pad, t - pad, r + pad, b + pad), fill=bg)
    d.text((x, y), text, font=font, fill=fill)
    return b + pad


def _card(base, lines):
    im = Image.alpha_composite(Image.fromarray(base).convert("RGBA"), Image.new("RGBA", (base.shape[1], base.shape[0]), (0, 0, 0, 160)))
    d = ImageDraw.Draw(im)
    y = 180
    for text, font, col in lines:
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
        self.clip = None             # (path, header, sub, note), set by the run_episode wrapper before build
        self.writer, self.n_frames, self.last, self.last_t = None, 0, None, 0.0

    def _cam_tf(self, ego_x):
        C = self.C
        loc = self.site.to_world(ego_x - self.cam_back, -0.5 * self.site.lane_w, self.map, z_off=self.cam_up)   # between the two lanes
        return C.Transform(loc, C.Rotation(pitch=self.cam_pitch, yaw=self.site.road_yaw))

    def _spawn_camera(self):
        bp = self.lib.find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(W))
        bp.set_attribute("image_size_y", str(H))
        bp.set_attribute("fov", str(self.cam_fov))
        bp.set_attribute("enable_postprocess_effects", "False")  # auto-exposure + bloom wash the untextured site out to white
        self.camera = self.world.spawn_actor(bp, self.C.Transform(self.site.origin.location, self.C.Rotation()))
        self.camera.listen(self.q.put)

    def _take(self, frame_id):
        """Called by the world proxy straight after every tick.  The image MUST be drained before any other RPC: in synchronous mode a
        pending camera image blocks the server's game thread until the client takes it, so a getter (bounding_box, get_transform)
        issued first deadlocks the server (reproduced on 0.9.16)."""
        self._img = None
        while True:
            try:
                im = self.q.get(timeout=20.0)
            except queue.Empty:
                return
            if im.frame >= frame_id:
                self._img = np.frombuffer(im.raw_data, dtype=np.uint8).reshape(im.height, im.width, 4)[:, :, 2::-1].copy()
                return

    def _emit(self, fr, t, v):
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
        y = _box(d, (24, 20), header, F_MID)
        _box(d, (24, y + 8), sub, F_SMALL)
        _box(d, (24, H - 60), f"t = {t:4.2f} s    ego speed = {v:5.2f} m/s", F_SMALL)
        self.writer.append_data(np.asarray(Image.alpha_composite(im, lay).convert("RGB")))
        self.last, self.last_t, self.n_frames = fr, t, self.n_frames + 1

    def build(self, spec):
        self.close()
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
            self.camera.set_transform(self._cam_tf(st["ego"].x))
        return st

    def step(self, controls):
        snap = super().step(controls)
        if self.camera is not None:
            ego = snap.actors["ego"]
            self.camera.set_transform(self._cam_tf(ego.x))        # takes effect on the next tick
            if self._img is not None:
                self._emit(self._img, snap.t, ego.v)
        return snap

    def finish(self, res, eps=0.0):
        if self.writer is None:
            return False
        _, header, sub, _ = self.clip
        vtxt, vcol = verdict(res, eps)
        lines = [(header, F_MID, (255, 255, 255)), (sub, F_SMALL, (225, 225, 225)), (vtxt, F_BIG, vcol)]
        if res.termination == "collision":
            lines.append(("episode ended at contact; the clip continues with the last controls held", F_SMALL, (210, 210, 210)))
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
            try:
                self.camera.stop()
                self.camera.destroy()
            except Exception:
                pass
            self.camera = None
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
        res = orig(plant, spec, policy, world, *a, **kw)
        if dsc:
            n_full = int(round(spec["horizon_s"] / plant.dt))
            for _ in range(n_full):                                # show what happens after an early stop (last controls held)
                if plant.n_frames >= n_full:
                    break
                plant.step({})
            if plant.finish(res, eps):
                written.append({"file": plant.clip[0], "policy": policy.name, "world": world.get("name"), "G": res.G,
                                "termination": res.termination, "verdict": verdict(res, eps)[0]})
            plant.clip = None
        return res

    run_episode._orig = orig
    harness.run_episode = run_episode
    return written
