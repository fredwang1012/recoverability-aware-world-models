"""T7: re-create an exported scene from its record and check that the stored label reproduces.

  python replay_scene.py ../results/dataset <id>                  # mock backend (runs now)
  python replay_scene.py ../results/dataset <id> --backend carla  # needs a live CARLA 0.9.16 server; NOT EXECUTED so far
Sensor-stream recording for Cosmos conditioning (segmentation / depth / HD-map) is NOT implemented yet.
"""
import argparse
import json

import core


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset")
    ap.add_argument("id")
    ap.add_argument("--backend", choices=["mock", "carla"], default="mock")
    ap.add_argument("--port", type=int, default=2000)
    a = ap.parse_args()
    rec = next(r for r in map(json.loads, open(f"{a.dataset}/scenes.jsonl")) if r["id"] == a.id)
    core.use_family(rec.get("family", "two_lane"))
    if a.backend == "mock":
        be = core.MockBackend()
    else:
        from carla_backend import CarlaBackend
        be = CarlaBackend(port=a.port)
    row = core.oracle(be, rec["theta"], full=True, cfg=core.MarginConfig(**rec["margin_config"]))
    print(f"stored V={rec['V']:.4f} ({rec['category']}) | replayed V={row['V']:.4f} ({row['category']}) on {be.name}")
    print("R replayed:", {k[2:]: round(v, 3) for k, v in row.items() if k.startswith("r_")})
    same = abs(row["V"] - rec["V"]) < 1e-9 and row["category"] == rec["category"]
    print("REPRODUCED" if same else "DIFFERS (expected if backend != record backend)")


if __name__ == "__main__":
    main()
