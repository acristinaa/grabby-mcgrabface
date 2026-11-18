# Level 2/3 brain track: QR codes + object recognition + Qwen (start here)

> Owner: Saad (started 2026-11-04). Branch: `l2-brain`. Everything runs on the laptop; nothing is installed on the robot.
> The start prompt for a Claude Code session is at the bottom of this file.

## Goal
A sorting brain that (1) recognizes what is on the table, (2) reads QR codes on two containers to know which container is
which and where it is, (3) decides where an item goes, (4) tells the grasp skill what to do, and (5) reports on a localhost
page, e.g. "The red ball is now in container 2".

## Architecture (agreed 2026-11-04)
```
User / SAP: "put the red ball into container 2"
        │
   Orchestrator (laptop): deterministic state machine
        ├─ look()        → Qwen-VL on ONE full-resolution head frame → [red ball, blue ball, ...] (JSON)
        ├─ read_boxes()  → OpenCV QR: container_1 = left, container_2 = right (+ corner pixels)
        ├─ decide        → plain rule: QR payload → container (the LLM never decides safety or motion)
        ├─ pick_place()  → grasp skill: stub now → sim teacher → SmolVLA prompt "put the red ball in the right box"
        ├─ check()       → sim truth now; later Qwen-VL / QR on a new frame
        └─ report()      → localhost page
   Qwen3 (text, local) turns a sentence into a validated tool call and writes the report text.
```
- **QR = plain OpenCV, no model.** `~/dev/g1-ext/.venv` has OpenCV 4.13 with `cv2.QRCodeDetector` and `cv2.QRCodeEncoder`.
- **Qwen runs locally** (Ollama or MLX on the Mac, not installed yet). Check current model names and licenses before choosing
  (Apache-2.0 or similar, because the code goes to SAP).
- **Resolution limit:** the VLA sees only 200 x 320 (`g1sim/policy_wire.py` `IMAGE_SHAPE`), too small for QR codes. The brain
  uses one full frame per task at 1280 x 800 (`g1sim/real_camera.py` `CAPTURE_W/H`). On the robot this needs a small "send one
  full frame" option in `real_camera` later (not part of this track; tell Finnegan when you need it).
- **The VLA never learns QR codes.** The orchestrator turns "container 2" into "left"/"right", so the VLA only learns which ball
  and which side from the sentence.

## What is already in the repo for you
| What | Where |
|---|---|
| 12 full-resolution sim head frames (1280 x 800) + ground-truth labels (name, shape, size, colour, position) | `tests/data/l2_brain/frame_0_*.jpg/.json` |
| Script that renders more of them (any scene, any seed) | `scripts/90_head_frames.py` |
| Scene in those frames | fixed `volleyball` scene: ball on a stand, blue basket, 1-3 distractors (box, can, red cylinder, purple cylinder, yellow block), random colours/light/textures |
| Lab View server pattern for the report page | `scripts/65_labview.py`, `g1sim/labview.py` |
| Camera constants, policy image format | `g1sim/real_camera.py`, `g1sim/policy_wire.py` |

Not yet in the scene: QR labels and a second container (the "world" track adds them). Until then, test the QR reader on
synthetic images and on the sample frames with a QR image pasted onto the basket (perspective warp with `cv2.warpPerspective`).

## Setup
- Sim env: `~/dev/g1-ext/.venv` (Python 3.12) with `requirements-record.txt`; the sim folder `~/dev/g1-ext/g1_brainco_sim`
  is built by `scripts/51_build_brainco_sim.py`. The hand meshes `models/revo2/*/meshes/` and `models/g1_brainco/assets/` are
  gitignored: if a build fails on missing meshes, get them from Finnegan (T7 or AirDrop).
- You only need the sim to render new frames. Steps 1, 2, 4 and 5 below work with the sample frames alone.
- Check: `~/dev/g1-ext/.venv/bin/python scripts/90_head_frames.py --count 3 --out runs/l2_head_frames` prints 3 lines.

## Steps (each with a check; stop and ask if a check fails twice)
1. **Local models:** install Ollama; pick a small Qwen3 text model (tool calling) and a small Qwen-VL model that run on the
   laptop. Report name, license, size and seconds per call.
2. **`g1sim/qr.py`:** make QR PNGs (payload `container_1` / `container_2`, high error correction) and decode images to payload +
   corner pixels. Unit tests with a "Proves / Catches" docstring. Check: decode rate over sizes, angles and blur; the smallest
   printable QR size that still decodes at 0.5-0.8 m in a 1280 x 800 head frame (number goes into the report).
3. **`look()`:** sample frame → Qwen-VL → validated JSON list of objects (name, colour, rough image position). Check: compare with
   the `.json` labels; then >= 95 % correct on 50 new frames from `90_head_frames.py`.
4. **Orchestrator** (`g1sim/orchestrator.py`): state machine find → read_boxes → decide → pick_place (stub) → check → report,
   Qwen3 tool call, timeouts, and a clear error report when anything is unsure (never guess). Unit tests per transition.
5. **Report page** on localhost (Lab View pattern): last task, what was seen, QR decision, result.
6. **Docs:** log entry in `PROGRESS.md`, plain-language explanation in `docs/LEARNING_PATH.md` §10.

Done for the first day: steps 1-2 pass, step 3 has a first accuracy number.

## Rules for this branch
- Do not touch the real-robot code (`scripts/84-89`, `g1sim/real_*`, `g1sim/arm_clearance*`) or the teacher
  (`g1sim/ball_grasp.py`): Finnegan works there.
- Commit on `l2-brain` after each step that passes its check; push only after asking Finnegan. Merge into `main` after a review.
- Code style: clean, readable code you can explain line by line; small functions; named constants with their source;
  docstrings that say why; few but meaningful tests.

## Start prompt (copy into a new Claude Code session)
```
We are working on the Unitree G1 robot project (SAP / CODE). Work only in my clone of
github.com/finneganadam-maker/UnitreeG1-Robot on the branch l2-brain (git fetch && git checkout l2-brain).
Read CLAUDE.md, then docs/L2_BRAIN_TRACK.md (my task, architecture, steps, rules), then the top of PROGRESS.md.
Reply to me in English. Work through the steps in docs/L2_BRAIN_TRACK.md in order, run each check, commit on
l2-brain after each passed step, and ask me before pushing. Do not touch the files listed under "Rules for this
branch". Start with step 1 and tell me which Qwen models you propose (name, license, size) before installing.
```
