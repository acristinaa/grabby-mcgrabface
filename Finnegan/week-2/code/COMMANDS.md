# COMMANDS.md — Start and stop everything

## ⚠️ The one rule

**Always first:**
```bash
cd ~/"Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot"
source .venv/bin/activate
```
The prompt then shows `(.venv)`. Forgotten = `ModuleNotFoundError`.
Needed in EVERY new terminal window.

**And:** windows (viewer) need `mjpython`, everything else `python`.

---

## 🟢 START

### Look at something (opens a window)
```bash
mjpython scripts/23_watch_table_demo.py    # project scene: G1 at the table  ← the usual one
mjpython scripts/33_watch_stand_env.py     # training environment (falls over)
mjpython scripts/21_watch.py               # any scene, neutral
mjpython scripts/00_view_g1.py             # bare G1, twitches
```
**Stop: just close the window.** Or `Ctrl+C` in the terminal.

### Without a window (runs to the end and is done)
```bash
python scripts/10_build_scene.py       # build the scene again (after changes)
python scripts/11_camera_check.py      # camera images -> docs/cam_*.png
python scripts/13_grasp_target.py      # pixel -> 3D grasp point (0.9 cm)
python scripts/22_record_video.py      # videos -> docs/videos/*.mp4
python scripts/20_benchmark.py         # How fast is the Mac?
python scripts/34_plot_curve.py        # learning curve -> docs/lernkurve.png
```

### Training (runs for a long time, NO window)
```bash
# Option A: in the terminal, you see everything live
python scripts/30_train_stand.py 5000000

# Option B: in the background, the terminal stays free
nohup python scripts/30_train_stand.py 5000000 > runs_stand.log 2>&1 &
```

---

## 🔴 STOP

| Situation | Command |
|---|---|
| Runs in the terminal (option A) | **`Ctrl + C`** |
| Window open | **Close the window** |
| Training in the background | `pkill -f 30_train_stand` |
| Viewer hangs | `pkill -f watch_` |
| **Everything, whatever it is** | `pkill -f "UnitreeG1-Robot.*scripts/"` |

---

## 🔍 CHECK

```bash
# Is anything running right now?
pgrep -fl "UnitreeG1-Robot.*scripts/"
# no output = nothing is running

# Follow the training live (Ctrl+C only stops the watching, not the training!)
tail -f runs_stand.log

# Latest status
tail -3 runs_stand.log

# Look at the results
open docs/                 # images
open docs/videos/          # videos
```

---

## 🧯 When something is stuck

```bash
pkill -f "UnitreeG1-Robot.*scripts/"    # kill everything
cd ~/"Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot"
source .venv/bin/activate          # activate again
python scripts/13_grasp_target.py  # test: does the base work?
```

| Error | Cause |
|---|---|
| `ModuleNotFoundError` | venv not activated |
| Window does not open / freezes | used `python` instead of `mjpython` |
| `command not found: mjpython` | venv not activated |
| Window not visible | It is behind VS Code — look in the Dock |

---

## 💾 Save your work

```bash
git status                    # what has changed?
git add -A
git commit -m "what I did"
git log --oneline             # all commits
git push                      # to GitHub (private repo)
```
Local under `~/Documents/…/SAP Robot Project/First Ideas/UnitreeG1-Robot`, saved in the **private** GitHub repo
[`finneganadam-maker/UnitreeG1-Robot`](https://github.com/finneganadam-maker/UnitreeG1-Robot).
⚠️ Contains BrainCo vendor files (`models/revo2/`) and internal SAP notes — before switching to
public, check the license and the contents.

## Web control panel (dashboard)

```bash
.venv/bin/python scripts/40_dashboard.py          # -> http://127.0.0.1:8000
.venv/bin/python scripts/40_dashboard.py --port 8011 --reload
```

What it can do:
- Live view, camera can be switched (free camera / `head_cam` / both wrist cameras).
  Dragging turns the free camera, the mouse wheel zooms.
- Sliders for all 41 actuators, grouped by arm / hand / waist / legs.
- Hands open/closed, everything to zero, pause, reset.
- **Set the hand mount live**: position and rotation per hand as sliders. The model
  is compiled again on every change (~180 ms). "Werte übernehmen" (apply values) prints the finished
  `DEFAULT_MOUNT` block, which belongs in `dashboard/scene.py`.
- Grip spread per hand live in cm — the same quantity that `16_hand_check.py` measures as its acceptance check.

## LeRobot G1 sim with walking controller (Stage 0.2, own environment)

Lives **outside** the repo under `~/dev/g1-ext`, so the project venv stays untouched
(LeRobot brings other numpy and mujoco versions).

**Start** — the G1 walks by command from code, no gamepad (~45 s; result 2026-09-14: 10 m without a fall).
To watch it, `--video` renders an mp4 to `docs/videos/stufe0_2_g1_laeuft.mp4`:
```bash
cd ~/"Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot"
DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib PYTHONUNBUFFERED=1 \
  ~/dev/g1-ext/.venv/bin/python scripts/50_spike_lerobot_walk.py --headless --video   # more: --speed --meters --timeout
open docs/videos/stufe0_2_g1_laeuft.mp4
```
Result: `runs/stufe0_2_lerobot_walk.json`. Stop: `pkill -f 50_spike`.

⚠️ **Do not use a MuJoCo window** (`mjpython`, without `--headless`): in this environment it freezes already **when it
opens** and leaves a process that cannot be killed — it only goes away when you log out and log in again
(status as of 2026-09-14). Orphaned camera processes then block port 5555: `lsof -nP -iTCP:5555 -sTCP:LISTEN`.

## BrainCo sim (Stage 0.3): G1 with our Revo2 hands

Same environment as above (`~/dev/g1-ext`). Short form for all commands:
```bash
cd ~/"Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot"
export DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib PYTHONUNBUFFERED=1
PY=~/dev/g1-ext/.venv/bin/python
```

| What | Command | Duration | Result |
|---|---|---|---|
| Build + check the sim (after changes to hands/mount) | `$PY scripts/51_build_brainco_sim.py` | ~15 s | `runs/stufe0_3b_brainco_sim.json`, `docs/stufe0_3b_brainco_haende.png` |
| Walk 10 m with BrainCo | `$PY scripts/50_spike_lerobot_walk.py --headless --brainco --video` | ~1.5 min | `runs/stufe0_3c_lerobot_walk_brainco.json`, video |
| Hands via DDS | `$PY scripts/52_brainco_hand_dds.py --video` | ~1 min | `runs/stufe0_3d_brainco_dds.json`, video |

The scripts clean up an orphaned camera process on port 5555 by themselves, and they stop with "SIM STEHT" (sim stalled) when the
sim time does not move on for 10 s. Never start two sim runs at the same time — they share the DDS channels.

## Grasp scene + openpi link (Stage 1.1 / 2.0)

```bash
cd ~/"Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot"
PY=~/dev/g1-ext/.venv/bin/python            # sim env
OPY=~/dev/g1-ext/openpi/.venv/bin/python    # openpi env
```

| What | Command | Duration | Result |
|---|---|---|---|
| Build + check the grasp scene | `$PY scripts/53_build_table_scene.py` | ~5 s | `runs/stufe1_1_greifszene.json`, `docs/stufe1_1_greifszene.png` |
| Check the openpi configuration | `$OPY -m openpi_g1.run check` | ~30 s | "KONFIGURATION OK" |
| Grasping without AI (Stage 1.4): 5 trials + video | `$PY scripts/54_grasp_scripted.py --video` | ~30 s | `runs/stufe1_4_greifen.json`, `docs/stufe1_4_greifen_handkamera.png`, `docs/videos/stufe1_4_greifen.mp4` |
| same, wrist pitch/yaw locked (G1 with 23 DoF) | `$PY scripts/54_grasp_scripted.py --lock-wrist --out runs/stufe1_4_greifen_23dof.json` | ~30 s | JSON, images named after `--out` |
| Evaluation (Stage 1.5b): 50 trials with random variation (`configs/randomization.json`) | `$PY scripts/54_grasp_scripted.py --randomize --trials 50 --out runs/stufe1_5b_alles.json` | ~1 min | success rate + failures by cause in the JSON |
| same, only one group (`objects`, `stand`, `physics`, `image`) | `$PY scripts/54_grasp_scripted.py --vary stand --trials 50 --out runs/stufe1_5b_stand.json` | ~1 min | same |
| Loop sim ↔ stand-in server, 1 episode | `$PY scripts/60_openpi_closed_loop.py` | ~25 s | `runs/stufe2_0_openpi_closed_loop.json`, `runs/stufe2_0_openpi/episode_000.npz` |
| Episodes → LeRobot dataset v2.1 | `$OPY -m openpi_g1.convert runs/…/*.npz` | seconds per episode | `~/.cache/huggingface/lerobot/g1_brainco/red_bottle_sim` |
| Normalization statistics (before training) | `$OPY -m openpi_g1.run norm-stats` | | `~/dev/g1-ext/openpi_g1/assets/…/norm_stats.json` |
| Look at the training configuration | `$OPY -m openpi_g1.run train --dry-run --exp-name X` | ~30 s | JSON |
| Training — **only Linux + NVIDIA GPU** | `$OPY -m openpi_g1.run train --exp-name X` | hours | `~/dev/g1-ext/openpi_g1/checkpoints/…` |
| Serve a checkpoint (GPU machine) | `$OPY -m openpi_g1.run serve --checkpoint DIR` | | port 8000; sim then: `60_openpi_closed_loop.py --no-server --host <GPU-machine>` |

The stand-in server has **no** π₀.₅ weights: it always answers "hold the ready pose" and only tests the link.
It shares port 8000 with the dashboard — do not start both at the same time. If something hangs: `pkill -f openpi_g1.run`,
check the port with `lsof -nP -iTCP:8000 -sTCP:LISTEN`.

## Lab View: watch grasp runs live and compare results

One small web page with the tabs Live (running runs: live image, trial, phase, success so far), History (all 54 runs with a
success-rate chart) and Training (placeholder). The page only reads files from `runs/labview/` and `runs/*.json`, so it can
stay open across many runs.

```bash
cd ~/"Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot"
# Terminal 1: start the page server (no MuJoCo), then open http://127.0.0.1:8030 (if taken: 8031-8039, see the output)
.venv/bin/python scripts/65_labview.py
# Terminal 2: a grasp run, slowed down to real time so you can follow the phases (--watch 1; without it a trial takes ~1 s)
~/dev/g1-ext/.venv/bin/python scripts/54_grasp_scripted.py --randomize --trials 5 --watch 1
```
Every 54 run writes lab view files unless you add `--no-labview`. Stop the server with Ctrl+C.

## Volleyball scene (Level 1)

Red ball on a tube stand (robot's left), basket on the robot's right; the check holds the ready pose 3 s and prints PASS/FAIL per check:
`~/dev/g1-ext/.venv/bin/python scripts/66_volleyball_scene_check.py [--watch 1]` → `docs/volleyball_scene_head.png`, `docs/volleyball_scene_overview.png`, lab view on http://127.0.0.1:8030

Two-hand pick-and-place without AI (sim teacher, `g1sim/ball_grasp.py`): waist turns to the ball, both palms cradle it,
all five fingers curl, lift, waist turns to the basket, release, then a 3 s "ball lies still in the basket" check.
```bash
# acceptance, nominal start (all trials must pass; ~2 s per trial)
~/dev/g1-ext/.venv/bin/python scripts/67_volleyball_pick_place.py --trials 10
# evaluation with ball + stand moved up to +-2 cm (result = success rate)
~/dev/g1-ext/.venv/bin/python scripts/67_volleyball_pick_place.py --trials 50 --jitter 0.02 --seed 0 --out runs/level1_volleyball_pick_place_jitter2cm_seed0.json
# watch one trial in real time in the lab view, and save it as a video
~/dev/g1-ext/.venv/bin/python scripts/67_volleyball_pick_place.py --trials 1 --watch 1 --video
```
Output: `runs/<name>.json`, `docs/volleyball_teacher_squeeze.png` (front + side view at the end of the squeeze), with `--video` `docs/videos/<name>.mp4`. `--no-labview` writes no lab view files.

Full domain randomization (2026-10-08): `--randomize` draws ball + stand (±10 cm), the robot's stand pose (±10 cm, ±10°),
physics, image and 1–3 distractors per trial; starts with the ball out of reach (0.33–0.52 m from the pelvis) are redrawn.
`--vary stand,physics` limits it to some groups.
```bash
~/dev/g1-ext/.venv/bin/python scripts/67_volleyball_pick_place.py --trials 50 --randomize --seed 0 --out runs/level1_volleyball_randomized_seed0.json
```

Walk world (2026-10-10, Level 1 Step 1): the free-standing robot in GR00T's walking crouch, pelvis held by a weld, standing at the
stop pose of the layout (0.458 m from ball and basket). `--randomize` draws ball/basket position, an arrival error (back 0-6 cm,
sideways ±4 cm, ±6°), physics and image (config group `"walk"`); `--map` runs a fixed grid of stances and layouts instead.
```bash
# random sweep (~1.2 s per trial without images; 200 trials ~4 min)
~/dev/g1-ext/.venv/bin/python scripts/67_volleyball_pick_place.py --scene volleyball_walk --randomize --trials 200 --seed 0 --no-labview --out runs/l1w_s3_sweep.json
# reach map (65 grid points)
~/dev/g1-ext/.venv/bin/python scripts/67_volleyball_pick_place.py --scene volleyball_walk --map --no-labview --out runs/l1w_s3_map.json
```
Walk-world squeeze image: `runs/<name>_squeeze.png` (the fixed scene keeps `docs/volleyball_teacher_squeeze.png`).

## Volleyball demo factory (Level 1 Step 5/6, 2026-10-14)
Records demos with the sim teacher (no-walk mode) to the T7. Plug the T7 in first (the script refuses an unmounted `/Volumes/...`).
Watch live: Lab View http://127.0.0.1:8030 → Live (`scripts/65_labview.py` must be running), one card per process.
```bash
# 5 test demos (Step 5), ~2-3 min
~/dev/g1-ext/.venv/bin/python scripts/69_volleyball_demo_factory.py --demos 5 --seed 100 --name smoke_v1 > runs/l1w_s5_smoke.log 2>&1
# dataset (Step 6): 4 processes, seeds 101-104 (never 100 = smoke, never 0/1 = eval), ~35-40 min, ~35 GB
for s in 101 102 103 104; do ~/dev/g1-ext/.venv/bin/python scripts/69_volleyball_demo_factory.py --demos 75 --seed $s --name level1_walkup_v1 > runs/l1w_s6_rec_$s.log 2>&1 & done
```
Output per demo: `episode_s<seed>_t<trial>.npz` (+ handover snapshot) and `.json` sidecar; per process `factory_s<seed>.json`; while running
`.lock_s<seed>`. The `._*` files are macOS metadata on exFAT, ignore them. Same seed + same folder = the same demos (deterministic).

## Convert the demos to LeRobot v3.0 + validate (Level 1 Step 6, 2026-10-14)
SmolVLA environment. The dataset goes to the internal disk (never onto the exFAT T7: macOS `._` files crash lerobot).
```bash
SV=~/dev/g1-ext/smolvla-test/.venv/bin/python
$SV scripts/79_convert_smolvla.py "/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/level1_walkup_v1" \
    --root ~/dev/g1-ext/datasets/g1_volleyball_walkup_v1 > runs/l1w_s6_convert.log 2>&1      # ~5 s per demo from the T7
$SV scripts/80_validate_dataset.py "/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/level1_walkup_v1" \
    --root ~/dev/g1-ext/datasets/g1_volleyball_walkup_v1 > runs/l1w_s6_validate.log 2>&1     # must end with "0 problems"
```

## Train SmolVLA (Level 1 Step 7, 2026-10-15)
Start long runs **detached** (`nohup ... & disown`): a run started from a Claude Code / terminal session dies when that window closes.
`caffeinate -i` keeps the Mac awake; keep it on power with the lid open. Watch it: Lab View http://127.0.0.1:8030/#training.
```bash
scripts/train_smolvla.sh smoke > runs/l1w_s7_smoke.log 2>&1                                    # 200 steps, ~4 min
nohup caffeinate -i scripts/train_smolvla.sh run1 > runs/l1w_s7_run1.log 2>&1 < /dev/null & disown   # 45k steps, ~11.5 h
nohup caffeinate -i scripts/train_smolvla.sh resume run1 >> runs/l1w_s7_run1.log 2>&1 < /dev/null & disown  # after a stop
```
Checkpoints: `~/dev/g1-ext/smolvla-runs/run1/checkpoints/<step>/pretrained_model` every 5k steps (~1.2 GB each).
Backup to the T7 (one verified .tar per checkpoint in `DATA Collection/smolvla-runs/<run>/`; never train or load from the T7):
`nohup scripts/backup_checkpoints.sh run1 --watch > runs/l1w_s7_backup_watch.log 2>&1 < /dev/null & disown`
Restore: `tar -xf checkpoint_<step>.tar` on the internal disk, then point `--checkpoint` / `--config_path` at it.

## VLA eval: policy server + sim client (Level 1 Step 4/8, 2026-10-15)
Server in the SmolVLA env, client in the sim env (TCP, `g1sim/policy_wire.py`). Gate commands and pass criteria:
`.planning/level1-walkup/STEP4-PLAN.md` section 3. Always kill the server afterwards.
```bash
SV=~/dev/g1-ext/smolvla-test/.venv/bin/python; PY=~/dev/g1-ext/.venv/bin/python; T7="/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos"
# pipeline proof: replay recorded demos through server + client
$SV scripts/81_smolvla_server.py --replay "$T7/smoke_v1" --port 8765 > runs/eval_server.log 2>&1 & SERVER=$!
$PY scripts/82_vla_eval.py --from-handover "$T7/smoke_v1" --trials 5 --name replay_smoke > runs/eval_replay.log 2>&1
kill $SERVER; wait $SERVER
# a trained checkpoint, held-out no-walk starts (after run1; MPS is refused while lerobot-train runs)
$SV scripts/81_smolvla_server.py --checkpoint ~/dev/g1-ext/smolvla-runs/run1/checkpoints/045000/pretrained_model --device mps --port 8765 > runs/eval_server.log 2>&1 & SERVER=$!
$PY scripts/82_vla_eval.py --no-walkup --seed 0 --trials 50 --server 127.0.0.1:8765 --name run1_45k_seed0 > runs/eval_run1_45k.log 2>&1
kill $SERVER; wait $SERVER
```

## Walk up to the table from random start situations (Level 4 step, 2026-10-08)

Each trial puts the table (and 0–3 obstacles) somewhere around the robot – table ahead, robot behind the table, robot
turned away, table beside, obstacle course, anywhere – then A* plans a path round everything, the robot walks it, docks at
the grasp pose of the volleyball scene and stands 1 s. Pass = no fall, no contact, docked in time, within ±10 cm / ±10°.
Arms are in the ready pose while walking. Trials chain: the next one starts where the last one ended.
```bash
cd ~/"Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot"
# Terminal 1 (optional): lab view server -> http://127.0.0.1:8030
.venv/bin/python scripts/65_labview.py
# Terminal 2: 12 trials (~10 min), watch http://127.0.0.1:8020 (camera + top view with table, obstacles, path)
DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib PYTHONUNBUFFERED=1 \
  ~/dev/g1-ext/.venv/bin/python scripts/68_walk_to_table.py --trials 12 --seed 0
# only some scenarios, e.g. the hard ones
DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib ~/dev/g1-ext/.venv/bin/python scripts/68_walk_to_table.py --scenarios behind_table,turned_away --trials 6
```
Output: `runs/level4_walk_to_table.json` (per trial: setup, checks, arrival error in the table frame, docking log, contacts),
`docs/level4_walk_to_table.png` (all trials in the table frame + arrival errors), with `--video` `docs/videos/level4_walk_to_table.mp4`.
Never together with another walking sim (DDS). `--no-live` / `--no-labview` switch the viewers off.

## Teleop: show demos yourself (Stage 2.1)

You guide the right hand in the browser. Every saved episode is a demo in the format of the demo factory.

| What | Command | Duration | Result |
|---|---|---|---|
| Start teleop (opens the browser) | `$PY scripts/57_teleop.py --randomize` | until Ctrl+C | `runs/teleop/<date_time>/episode_NNN.npz` + `.json` |
| same with iPhone (app "MuJoCo AR") | `$PY scripts/57_teleop.py --randomize --phone` | | IP:port is shown in the terminal |
| Acceptance check without humans: simulated operator, server, iPhone protocol, conversion | `$PY scripts/58_teleop_check.py` | ~2 min | `runs/teleop_check.json`, "ABNAHME BESTANDEN" |
| Teleop episodes → LeRobot v2.1 (only successful ones) | `$OPY -m openpi_g1.convert runs/teleop/<folder>/episode_*.npz --repo-id g1_brainco/teleop` | seconds per episode | `~/.cache/huggingface/lerobot/g1_brainco/teleop` |

Keys: `W/S` forward/back · `A/D` left/right · `E/Q` up/down · `J/L`, `I/K`, `U/O` rotate · `Shift` fine · `G` grasp posture · `Space` grasp · `H` ready pose · `Enter` save · `⌫` discard · `1/2/3` view.
The page shows the sequence and the status. While recording, the sim runs at ~0.5× real time (three camera images per step).
If you leave the page, the hand stops. Port 8010; another one with `--port`. iPhone: Mac and phone in the same Wi-Fi,
start the app with the phone pointing forward; hold the button = the hand follows, switch = grasp. On the first run macOS asks
whether Python may accept incoming connections → allow it.

## Learning path L1/L2: walking with feedback, map and path (2026-10-02)
Watch live in the browser: http://127.0.0.1:8020 (if the port is taken, the view uses 8021, 8022 … – the address is in the output).
```bash
cd ~/"Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot"
# L1 go to point: 10 random goals (~3 min), top view docs/lernpfad_l1_wege.png
DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib ~/dev/g1-ext/.venv/bin/python scripts/61_goto_check.py [--video]
# L2 map + A*: 10 room changes (~4–5 min), map docs/lernpfad_l2_karte.png
DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib ~/dev/g1-ext/.venv/bin/python scripts/62_rooms_check.py [--video]
```
Never run two walking sims at the same time. If a run hangs and cannot be killed (graphics driver), the process only goes away when you log out.

## Tests

```bash
cd ~/"Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot"
scripts/run_tests.sh          # unit + integration, all three environments (~2-3 min)
scripts/run_tests.sh all      # plus end-to-end: connect, hands via DDS, walk 10 m (~8-10 min)
scripts/run_tests.sh all -k laeuft   # only tests whose name contains "laeuft"
```

| Level | File | Checks |
|---|---|---|
| Unit | `tests/test_unit_brainco.py`, `tests/test_unit_mount.py`, `tests/test_unit_action_space.py` | hand normalization, motor order, mount math against the Unitree URDF, 26-D layout |
| Unit (openpi) | `tests/test_unit_openpi_policy.py` | cameras → π₀.₅ image inputs, delta actions and back |
| Integration | `tests/test_integration_table_scene.py`, `tests/test_integration_brainco_model.py`, `tests/test_integration_sim_env.py` | build models, motor mapping, hand over DDS, `reset()` during steps |
| Integration | `tests/test_integration_table_env.py`, `tests/test_integration_openpi_env.py` | grasp scene: ready pose free, holding without sag; openpi client episode |
| Integration | `tests/test_integration_grasp_ik.py` | IK reaches the grasp pose at the bottle, with and without wrist pitch/yaw; arm stays out of the trunk; no jumps |
| Integration | `tests/test_integration_randomization.py` | random variation: reset restores everything bit-exact, repeat, seed, only the bottle is red |
| Integration | `tests/test_integration_teleop.py` | teleop: thumb first, then fingers; hand never opens by itself; speed, workspace, dead-man switch, iPhone direction |
| Integration (openpi) | `tests/test_integration_openpi.py` | configuration, stand-in policy through openpi's transforms, dataset v2.1 + openpi's data loader, norm statistics |
| End-to-end | `tests/test_e2e_sim.py` | LeRobot robot + walking controller: connect 3×, hands via DDS, walk 10 m |

**Set up again** (once):
```bash
brew install cmake
mkdir -p ~/dev/g1-ext && cd ~/dev/g1-ext
git clone https://github.com/huggingface/lerobot.git
git -C lerobot checkout 89236ea0   # LeRobot 0.6.2 as of 2026-09-14, the state 0.2/0.3/L1/L2 were checked against (re-setup 2026-10-08)
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git
git clone --depth 1 -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git

# CycloneDDS is not available via Homebrew -> build it yourself
cd cyclonedds && mkdir -p build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/dev/g1-ext/cyclonedds/install -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTING=OFF -DENABLE_SSL=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build . --target install -j 10

cd ~/dev/g1-ext
/opt/homebrew/bin/python3.12 -m venv .venv
CYCLONEDDS_HOME=$HOME/dev/g1-ext/cyclonedds/install .venv/bin/pip install -e unitree_sdk2_python
.venv/bin/pip install pyyaml loguru opencv-python pyzmq msgpack msgpack-numpy matplotlib scipy
.venv/bin/pip install -e "lerobot[unitree_g1]"
.venv/bin/pip install mink==1.3.0 qpsolvers==4.13.0 daqp==0.9.1   # IK for grasping (Stage 1.4)
```

| Pitfall (macOS) | Fix |
|---|---|
| `cyclonedds` is missing in Homebrew | build it yourself (above); CMake 4 needs `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` |
| LeRobot and the sim set up DDS on `lo` (Linux name), hard-coded | the script sets up `lo0` first — the channel factory is a singleton |
| The sim needs `scipy`, which is not in its requirements | install it as well |
| MuJoCo window (`mjpython`) freezes on opening, process cannot be killed | `--headless`; to watch, `--video` |
| Without a window: the elastic band crashes in the 1st step ("zero norm quaternions") | `--headless` calls `mj_forward` first |
| Camera port 5555 taken | stop the orphaned camera process (see above) |
| Process hangs after the end (DDS threads) | the script exits hard with `os._exit` |
| The `timeout` command does not exist on macOS | — |
| `lerobot[unitree_g1]` pulls numpy down (2.5 → 2.2.6 on 2026-10-08) | fine: the volleyball/randomization tests and 67 still pass with it |
| cv2 and pygame both bring SDL2 ("Class SDL_… is implemented in both") | harmless warning, ignore |
