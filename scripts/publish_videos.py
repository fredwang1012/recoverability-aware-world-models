"""
Publish the case videos: compress the clips recorded by scripts/carla_videos.py (about 50 MB each, near-lossless) into small H.264 copies
under docs/videos/ (about 1 MB each) and write docs/videos/README.md from the recording index.

    python scripts/publish_videos.py            (repo root; no simulator)

Reads  mining/results/carla/videos/*.mp4 and meta/*.json   (local only, git-ignored)
Writes docs/videos/*.mp4 and docs/videos/README.md          (committed)
Visualisation only: the table repeats the recorded outcome of each clip; it never sets a label.
"""
import glob
import json
import math
import os
import re
import subprocess
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
SRC = os.path.join(ROOT, "mining", "results", "carla", "videos")
DST = os.path.join(ROOT, "docs", "videos")
CAT = os.path.join(ROOT, "mining", "cases", "catalogue.yaml")


def titles():
    out, cur = {}, None
    for line in open(CAT, encoding="utf-8"):
        m = re.match(r"- id: (RC\d+)", line)
        if m:
            cur = m.group(1)
        m = re.match(r"  title: (.*)", line)
        if m and cur:
            out[cur] = m.group(1).strip()
    return out


def ffmpeg():
    import imageio_ffmpeg
    return imageio_ffmpeg.get_ffmpeg_exe()


def compress(src, dst, exe):
    subprocess.check_call([exe, "-loglevel", "error", "-y", "-i", src, "-c:v", "libx264", "-preset", "slow", "-crf", "26",
                           "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-an", dst])


def main():
    os.makedirs(DST, exist_ok=True)
    exe = ffmpeg()
    rows = []
    for mp4 in sorted(glob.glob(os.path.join(SRC, "*.mp4"))):
        name = os.path.basename(mp4)
        dst = os.path.join(DST, name)
        if not os.path.exists(dst) or os.path.getmtime(dst) < os.path.getmtime(mp4):
            compress(mp4, dst, exe)
        meta = json.load(open(os.path.join(SRC, "meta", name[:-4] + ".json")))
        case = name.split("_")[0]
        g = meta["G"]
        g = "collision / -inf" if g in ("-inf", None) or (isinstance(g, float) and not math.isfinite(g)) else f"{float(g):+.3f}"
        rows.append((case, name, meta["policy"], meta.get("world") or "", meta["verdict"], g))
    t = titles()
    L = ["# Case videos\n",
         "36 clips of 10 to 15 s, one per move in each case's headline scene, re-run on CARLA with a chase camera. They are a visual aid only: "
         "labels come from the evidence in the records, never from these runs. Blue = ego, red = parked car, orange = moving car; the road and "
         "grass are painted in afterwards because the generated map has no scenery. A car the ego has not been told about yet (RC10, RC11, RC13, RC14) "
         "is drawn as a faint ghost marked \"not visible to ego\" until its reveal time. A run ends at first contact and the clip freezes on a "
         "CONTACT banner. Each clip starts with a 1 s title card and ends with a 1 s result card. The RC04 clips are the closest attempt, not a "
         "witness: no RC04 scene on CARLA shows the staged move working where both simple moves fail.\n",
         "Click a file name to open the clip on GitHub. Margin G above 0 means the move keeps the required clearance; the headline margins are only "
         "stable to about 0.2 across server sessions (see the [report](../REPORT.md#results-by-case)).\n",
         "| Case | Clip | Move | World | Outcome | G |", "|---|---|---|---|---|---|"]
    for case, name, policy, world, verdict, g in rows:
        L.append(f"| {case} {t.get(case, '')} | [{name}]({name}) | `{policy}` | {world} | {verdict.split(':')[0].lower()} | {g} |")
    L.append("\nRebuild: `cd mining && python ../scripts/carla_videos.py --force` (needs the CARLA server, see [CARLA_SETUP](../CARLA_SETUP.md)), "
             "then `python scripts/publish_videos.py` from the repo root. The uncompressed originals stay in `mining/results/carla/videos/` (local only).")
    open(os.path.join(DST, "README.md"), "w", encoding="utf-8", newline="\n").write("\n".join(L) + "\n")
    print(f"{len(rows)} clips -> {DST}")


if __name__ == "__main__":
    sys.exit(main())
