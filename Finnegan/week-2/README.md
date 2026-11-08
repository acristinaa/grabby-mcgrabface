# Finnegan – week 2 (8 Nov 2026)

## Week 2 in short

**We trained our VLA (SmolVLA) on the PC at CODE, and now we have to test it.**

- The robot's "brain" for grasping is SmolVLA, a small vision-language-action model (~450 M parameters). It looks through the head
  camera, gets the sentence "put the ball in the basket", and moves the arms, hands and waist itself.
- This week it was trained on the **PC at CODE** (GTX 1060) on **1,691 demonstrations** recorded in simulation. Training took
  **50 hours** and finished on **Fri 6 Nov at 05:49**. The model files are on the PC.
- **It is not tested yet.** Next step: let it grasp in the simulation 100–200 times on situations it has never seen and count the
  successes. Our first, smaller model (run1, trained on 300 demos) managed **72 %**. The new one has to beat that.
- In parallel I built the software that moves the real arms **safely** at SAP (13 Nov), and prepared the QR-code / Qwen track for Saad.

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

## What is open before SAP (13 Nov)

1. **Test run2** on the PC (100–200 grasps in the simulation) and pick the best version. ← next
2. Decide a few safety details for the arm movement (e.g. after a lost connection: hold the arms or bring them home?).
3. Test the chosen model through the whole real-robot software in simulation before it touches the robot.
4. Prepare the laptop that runs the model at SAP (the model runs on the laptop, the robot only sends camera + joint data).

**Visit 1 (13 Nov):** robot at the table → safe routes lift the arms → the VLA grasps and places on its own → safe routes back.
**Visit 2 (later):** the robot walks up to the table itself, then grasps and places.

Honest: grasping that works in simulation often works much less on a real robot at first. Visit 1 is about running the whole chain
safely and learning what is different on the real robot.

---

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
