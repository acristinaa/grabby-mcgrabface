# Finnegan – week 2 (8 Nov 2026)

## Week 2 in short (updated 8 Nov, evening)

**We trained our VLA (SmolVLA) on the PC at CODE and tested it. It is not good enough yet, so it is training further right now.**

- The robot's "brain" for grasping is SmolVLA, a small vision-language-action model (~450 M parameters). It looks through the head
  camera, gets the sentence "put the volleyball in the basket", and moves the arms, hands and waist itself.
- **Trained (run2):** on the PC at CODE with **1,691 demonstrations**, 90,000 steps, **50 hours** (4–6 Nov).
- **Tested (8 Nov):** in 100 new situations in the simulation it succeeded **47 times (47 %)**. The programmed teacher: 100/100.
  On the old, easier test conditions run2 got 54 %, our first model run1 72 %.
- **Why so low:** our planning mistake. The training length was planned for 800 demos; when we raised the demos to 1,691 we did not
  make the training longer. So the model saw each example only ~0.6 times (run1: 1.65 times). The test says it clearly: it even fails
  on situations it trained on (3/10) = **undertrained**, not "bad data".
- **Now (run3):** the PC continues training from run2 for another 150,000 steps (~77 h), so the model sees every example ~1.6 times.
  **Done ~Wed 11 Nov evening.** Mid-check on Tuesday; if it does not improve or it gets tight, we rent a bigger GPU.
- **Safety (in build):** a "virtual fence" that checks every arm command before it is sent: no hand/finger closer than a few cm to the
  table, stand, basket or the robot's own body, otherwise the command is not sent and the arms stop. Plus a check that the movements
  are not too jerky. (In the test, 16 of 100 trials touched the table/stand/basket - the fence is for exactly that.)
- **Goal for SAP (13 Nov):** the robot runs the learned grasp, does not hurt itself or hit the table, moves smoothly, and tries to pick
  up the ball. A miss is OK. Training target stays 85 % in the simulation.

---

## What we built in week 2

### 1. Training data, recorded on the PC at CODE
- A programmed "teacher" does the task in simulation: walk up position, two-hand grasp of the volleyball, put it in the basket.
  Its runs are the examples the VLA learns from.
- To make the VLA robust, every demo is different: table height (70–75 cm), where the robot stands, where ball/stand/basket are,
  light, table textures, ball weight and friction, motor strength, delay of the motors, camera position, and 5 different ways of
  saying the task.
- **1,691 demos** recorded on the PC (3 recorders in parallel), converted to the LeRobot format and checked: **0 problems**.

### 2. The VLA trained on the PC at CODE (run2)
| | run1 (October, MacBook) | **run2 (this week, PC at CODE)** |
|---|---|---|
| Demos | 300 | **1,691**, much more varied |
| Training | 45,000 steps | **90,000 steps, 50 h** |
| Result in the simulation | **72 %** on new situations | **not tested yet** |

The finished model (865 MB) and 9 in-between versions (every 10,000 steps) are on the PC.

### 3. Safe arm movement for the real robot (for SAP on 13 Nov)
- At SAP the robot stands at a table. Its arms hang down, so a straight move to the start pose would sweep the hands through the table.
- Built: fixed, checked routes ("waypoints"): arms up beside the body first, then forward to the start pose. Every route is checked
  in a 3D model of robot + table to stay at least 5 cm away from table, stand and basket.
- Then the VLA takes over and grasps on its own. Afterwards the arms go back along the same safe routes.
- If something goes wrong (no data from the robot, an arm touches something, Ctrl+C), the arms stop or go home along the safe
  routes; they never just drop into the table.
- Everything is tested in a simulation of the robot's arm interface. Each step was checked by an independent review.

### 4. Level 2 prepared for Saad
QR codes on two boxes + a local Qwen model as the "brain" that decides which box ("put the red ball in box 2"). Plan, sample
camera pictures and a start prompt: [`code/docs/L2_BRAIN_TRACK.md`](code/docs/L2_BRAIN_TRACK.md).

---

## Plan until SAP (13 Nov)

| Day | Training (PC at CODE, runs by itself) | Safety / robot software (Finnegan + Claude) |
|---|---|---|
| Sun 8 Nov | run3 started | virtual fence + smoothness: plan, review, build |
| Mon 9 Nov | trains | fence: review + test in the simulation against the 16 contact cases |
| Tue 10 Nov | trains; **mid-check** (test an in-between version) -> stay on the PC or rent a GPU | smoothness check in the robot rehearsal; robot-day checklist updated |
| Wed 11 Nov | done ~20:00 -> **final test** (100+ situations), pick the best version | laptop for SAP ready (self-test 89) |
| Thu 12 Nov | - | **rehearsal**: chosen model + fence + full robot software in the simulation |
| Fri 13 Nov | - | **SAP Potsdam** |

Open decisions for Finnegan: what the arms do per fault (hold or go home), the waist limit, fence distances.
Later (after the basics work): Qwen says the sentence ("please put the ball in the basket") -> the VLA does it (see below).

**Visit 1 (13 Nov):** robot at the table → safe routes lift the arms → the VLA grasps and places on its own → safe routes back.
**Visit 2 (later):** the robot walks up to the table itself, then grasps and places.

Honest: grasping that works in simulation often works much less on a real robot at first. Visit 1 is about running the whole chain
safely and learning what is different on the real robot.

---

## Qwen later: "Hey robot, put the ball in the basket"

Qwen does not move the robot. It turns what a person says into one of the sentences the VLA has learned (5 phrasings of "put the
volleyball in the basket"), and the operator confirms before anything moves. Needed: a local Qwen model (Ollama), a small orchestrator
(Saad's track, `code/docs/L2_BRAIN_TRACK.md`), and a small change so the robot program takes that sentence instead of a fixed one.
Estimate: 1-2 days once the grasp works. Sorting into two boxes ("box 1 / box 2" via QR codes) needs new demos and training = Level 2.

## How to reach the PC at CODE

The PC is in our Tailscale network, so it is reachable from anywhere (ask Finnegan to add your SSH key).

```bash
ssh grabby@<pc-tailscale-ip>
```

To copy the finished model to a laptop (run each line on its own, without anything after it):

```bash
rsync -avP grabby@<pc-tailscale-ip>:dev/g1-ext/smolvla-runs/run2/checkpoints/090000/pretrained_model/ ./run2_090000/
```

The training data (137 GB) stays on the PC. Only the model and the test results get copied.

---

## What is in this folder

- [`code/`](code/): my whole project code as of 6 Nov (copy of my repo `UnitreeG1-Robot`).
- Read first: [`code/PROGRESS.md`](code/PROGRESS.md) (top part) · the robot-day plan [`code/docs/ROBOT_DAY_RUNBOOK.md`](code/docs/ROBOT_DAY_RUNBOOK.md) ·
  the safe arm routes [`code/.planning/real-robot/SAFE-PATHS.md`](code/.planning/real-robot/SAFE-PATHS.md) ·
  explanations of every concept [`code/docs/LEARNING_PATH.md`](code/docs/LEARNING_PATH.md) (section 10).
- Older planning notes in `code/.planning/` and `code/docs/` show how we got here; some describe plans we later changed.
