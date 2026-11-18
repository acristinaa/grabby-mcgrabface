# Finnegan – week 3 (18 Nov 2026)

`code/` is the full current state of the robot code (week 2's copy stays untouched in `Finnegan/week-2/code`).

## New since week 2

- **Model run3 (150,000 steps)**, trained on the PC on the 1,691 v2 demos (continued from run2's 90k weights).
  Simulation test: **68 / 100** new situations (v1 37/50, v2 31/50); on the harder v2 scenes with the safety fence on: **30 / 50**.
  Training command: `code/scripts/train_smolvla.sh`; data recording and conversion: `code/scripts/run2_pc.sh`,
  `69_volleyball_demo_factory.py`, `79_convert_smolvla.py`, `80_validate_dataset.py`; evaluation: `82_vla_eval.py`.
- **Demo envelope** (`g1sim/real_safety.py`): every arm command is clipped to the joint range seen in the v1 + v2 demos.
- **Done check** (`g1sim/done_check.py`): ends the model's turn after one grasp-and-place (grip -> release -> hands relaxed 1 s),
  then the arms go home on the safe path. In the simulation it stopped all 30 successes with the ball in the basket.
- **Flight recorder** (`scripts/91_watch.py`): read-only, runs in a second terminal next to 84/86 and records once per second
  which layer is healthy - network, robot state stream, hands, other programs sending (arm fight, navigator), arm following,
  robot moving. `ALERT` lines + a summary say which layer failed first.
- Robot-day self-test (`scripts/89_robot_day_selftest.py`) and runbook (`code/docs/ROBOT_DAY_RUNBOOK.md`) on run3.

## Put it on the robot laptop (Apple Silicon Mac or Ubuntu x86_64)

The code runs on the laptop. The laptop talks to the robot over the Ethernet cable; nothing is installed on the robot
(SSH to the robot PC is only used for the head camera, `84 --robot-pc`).

1. **Code** - the motion lock needs the code at the top of its own git repo, so copy it out and commit it once:
   ```
   git clone --single-branch -b week-3 https://github.com/acristinaa/grabby-mcgrabface.git ~/grabby
   cp -R ~/grabby/Finnegan/week-3/code ~/UnitreeG1-Robot
   cd ~/UnitreeG1-Robot && git init -q && git add -A && git commit -qm "robot day code"
   ```
2. **Python environments** (internet, 20-60 min): `bash scripts/setup_robot_laptop.sh --robot-env --policy-env`
3. **Model - not in this repo** (906 MB; GitHub allows 100 MB per file). Copy the run3 folder from Finnegan (USB stick) to
   `~/g1-robot-day/smolvla-runs/run3/checkpoints/150000/pretrained_model`
   (`model.safetensors` sha256 `ef86fffee140e065e0e19625e125af3dc1c1a9d4d0ae1384983c90d5221db931`).
4. **Self-test**: `source ~/g1-robot-day/robot-env.sh && python scripts/89_robot_day_selftest.py` -> `SELFTEST: PASS`.
5. **Motion unlock**: only Finnegan, on the day (runbook / solo page step 5). Without it 86 only does read-only checks.
6. **On site**: second terminal `python scripts/91_watch.py --iface <IF>`, then follow the runbook.

Not in the repo: model weights, datasets (many GB), Python environments, run logs.
