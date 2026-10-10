# Week 1: G1 walk + pick-and-place in MuJoCo

Source: [supat-roong/g1-manipulation-challenge](https://github.com/supat-roong/g1-manipulation-challenge), a fork of [luckyrobots/g1-manipulation-challenge](https://github.com/luckyrobots/g1-manipulation-challenge). No upstream license, all credit to the authors.

The G1 walks to a table, grasps a red cylinder with its right hand, carries it and places it upright on a second table. Walking uses pretrained ONNX policies, and the arm uses scripted IK. Chosen because it runs natively on a MacBook M1 Pro (no NVIDIA GPU, Docker or ROS 2).

`sim/` holds only the run-critical files: `run.py`, `ik/`, `eval/sweep.py`, scene, model config and 4 policies. `assets/` (meshes, 138 MB) is git-ignored. Copy it from upstream.

## Run (from `sim/`)
    python3 -m venv .venv && source .venv/bin/activate
    pip install mujoco onnxruntime numpy opencv-python
    python -c "from ik import pipeline as e; print(e.run_once(video=None)['success'])"
    python eval/sweep.py -n 10 --seed 1 --preset shipped -j 4
    mjpython run.py --no-cameras    # viewer

## Findings
- Headless episode on M1 Pro: success (placed).
- macOS: the viewer needs `mjpython`, and the camera pop-ups crash OpenCV, so use `--no-cameras`.
- In the viewer the robot drifts with no input. A headless run with no input stands still. Cause not found yet.
- Uses the cylinder's true position. Cameras are unused. Simulation only.