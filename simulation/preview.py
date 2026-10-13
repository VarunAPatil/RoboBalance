"""Open a Wall-E MuJoCo model paused or simulate it unpowered under gravity.

Examples:
  .venv/bin/python -m simulation.preview
  .venv/bin/python -m simulation.preview --model simulation/models/wall_e_balance.xml --simulate
  MUJOCO_GL=egl .venv/bin/python -m simulation.preview --offscreen --camera battery_side
"""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path


DEFAULT_MODEL = Path(__file__).resolve().parent / "models/wall_e_preview.xml"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--camera", choices=("three_quarter", "battery_side"), default="three_quarter")
    parser.add_argument("--offscreen", action="store_true", help="Save a PNG instead of opening a window")
    parser.add_argument("--output", type=Path, help="PNG path for --offscreen")
    parser.add_argument("--simulate", action="store_true",
                        help="Advance the model under gravity with zero actuator commands")
    parser.add_argument("--seconds", type=float, default=2.0,
                        help="Simulation duration before an offscreen render (default: 2 s)")
    args = parser.parse_args()
    if args.output and not args.offscreen:
        parser.error("--output requires --offscreen")
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    if args.offscreen:
        os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco

    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        parser.error(f"Model not found: {model_path}")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if args.simulate and args.offscreen:
        steps = round(args.seconds / model.opt.timestep)
        for _ in range(steps):
            mujoco.mj_step(model, data)
        print(f"Simulated {data.time:.3f} s under gravity with zero actuator commands; "
              f"contacts={data.ncon}")
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, args.camera)
    if camera_id < 0:
        parser.error(f"Camera not found: {args.camera}")
    if args.offscreen:
        from PIL import Image

        output = (args.output or Path(__file__).resolve().parent / "renders" / f"wall_e_{args.camera}.png").resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        with mujoco.Renderer(model, width=1280, height=960) as renderer:
            renderer.update_scene(data, camera=args.camera)
            Image.fromarray(renderer.render()).save(output)
        print(f"Rendered {args.camera}: {output}")
        return
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        parser.error("No desktop display found; use --offscreen or run on a graphical desktop")
    import mujoco.viewer

    mode = "unpowered gravity simulation" if args.simulate else "paused inspection"
    print(f"Opening MuJoCo {mujoco.__version__} viewer ({mode}): {model_path}", flush=True)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = camera_id
        viewer.sync()
        print("Wall-E viewer is open. Close the viewer window to exit.", flush=True)
        while viewer.is_running():
            if args.simulate:
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(model.opt.timestep)
            else:
                time.sleep(0.1)


if __name__ == "__main__":
    main()
