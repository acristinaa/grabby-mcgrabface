# History — what was built, in order

The project started in the team repo `finneganadam-maker/UnitreeG1-Robot` (owner: Finnegan Adam). Finnegan built the
simulator setup, the volleyball scene and the scripted two-hand pick-and-place teacher. This repo continues Saad's part from
2026-10-08: distractors, randomization, walking to the table, and walking + grasping in one sim. Numbers are in
[RESULTS.md](RESULTS.md).

---

## 2026-10-08 (Wed)

### Distractors on the volleyball table
- **Why:** if the ball is the only object, a VLA can learn "grab whatever is there". Real tables have clutter.
- 5 distractors (`table_scene.VOLLEYBALL_DISTRACTORS`): cardboard box, drink can, red cylinder, purple cylinder, yellow block.
  1–3 per trial, placed clear of the ball, the basket and the arc the ball travels while it is carried; the rest are parked on
  the floor far away. Touching one counts as a failed demo.
- New randomization group `distractors`, `67 --distractors`. **30/30.**

### Full randomization in the volleyball scene
- Every randomization group now works in the volleyball scene: object offsets, robot stand pose, physics, image.
  Per-scene ranges in `configs/randomization.json` (`scenes` block).
- The teacher follows the robot's *real* stand pose: it turns the waist by however far ball and basket moved around the waist
  axis, and builds hand orientations from pelvis yaw + waist.
- **Reach rule:** all groups together gave 85/100 — a shifted stand plus a shifted ball put the ball out of reach. Starts with
  the ball outside 0.33–0.52 m from the pelvis are redrawn. **50/50 + 50/50.**

### Walking to the table from random situations
- `g1sim/approach.py` (scenarios, path planning, docking), `g1sim/approach_world.py` (walk world with a movable table with legs
  and obstacles), `scripts/68_walk_to_table.py`.
- The LeRobot walking stack (LeRobot 0.6.2, CycloneDDS, unitree_sdk2) was set up on the Mac again.
- Found and fixed: fingertips hit the table at the zero arm pose; GR00T barely moves at very slow commands (dead band
  measured and compensated); the live view slowed the control loop and made docking oscillate (controller now on sim time);
  a bus error in the live view's marker drawing (uninitialized memory).
- **24/24 + 24/24.**

## 2026-10-09 (Thu)

### Walking, calmer and more natural
- The walk "looked funky": the robot strayed 9–10 cm from its path. Path pulled taut and smoothed, speed profile, pure-pursuit
  follower; GR00T drifted 35 cm sideways in a 180° turn on the spot, so the robot now steps back while it turns.
  Path deviation **9.5 → 4.4 cm**.
- **Checkpoint** 0.75 m before the table: align there, raise the arms, then walk **straight in** with forward steps only
  (before, the body turned up to 87° away from the table while side-stepping). Arms down with a slight natural swing while walking.

### Merge with Finnegan's new pick and place
- Finnegan pushed a new teacher (`e5a2f31`): natural start pose (elbows beside the body), one straight smooth reach, retract and
  turn back to the front at the end; the waist became part of the action (27 values instead of 26).
- Merged into this work: 7 files conflicted, resolved by keeping both sides (his motion + the random-start handling and the
  distractors). The basket check now works in the basket's frame (the walk world turns the table).
- Randomized: 46/50 (the straight reach let the right wrist touch the basket on some starts), 50/50 on the other seed.

### Walk + pick and place in ONE continuous sim
- The ruling: *once GR00T has got the robot to the table, switch it off and start the pick-and-place code.*
- `g1sim/walk_grasp.py`, `scripts/69_walk_pick_place.py`. GR00T walks; at the table the sim thread is paused (its step lock is
  held), so GR00T's torques no longer reach the robot. The pelvis is welded where it stands, all joints are held, and the
  teacher runs on the very same physics state. Then GR00T takes over again and walks to the next table.
- The walk world got the ball (free body), stand and basket (fixed to the movable table) and the pelvis hold.
- **Found:** GR00T stands in a crouch (pelvis 0.750 m instead of 0.793 m, leaning back ~2.5°). From the old stop point the
  hands hit the table and stand. Stopping 5 cm further back fixed it (crouch sweep 9/9) → first full run **1/1**, then **5/6**
  and **6/6** (+ a 4/6 rerun): every failure had docked ~3 cm sideways.

### Waist experiment — rejected by measurement
- Question: is it safer for the real robot to turn the waist less? Limits 25° … 0° all failed (the arms cannot hug a ball 35°
  off the torso axis), and the centre of mass moved *more*. The full waist turn stays; the real risk is whether the real G1 lets
  us command the waist while it balances — to be tested on the robot.

### Grasp made robust to where the walk stops
- `scripts/70_handover_sweep.py` replays the handover offline from any dock offset in seconds (no 10-minute walk run).
  Before: **13/25**. Causes: hands turning through the ball when too close; the stretched arm sagging 7 cm under gravity when
  too far.
- Fixes in `ball_grasp`: the hands rise 6 cm while the waist turns (`FACE_LIFT`); **gravity offset** — each arm target is aimed
  higher by exactly the static sag of the joint controller at that pose (gravity torque / kp).
- After: sweep **35/35** and **36/36** (±5 cm, ±3°), randomized pick-and-place **50/50 + 50/50**, continuous walk + pick and
  place **12/12** (docked up to 4.7 cm sideways).

### This repository
- Created from the working state: only the code the volleyball pipeline needs (dependency closure of the scripts), tested on
  its own (144 tests passed, all four pipelines ran from here).

---

## Open points
1. **Distractors in the continuous walk + grasp run** (next step).
2. **Gravity offset in the recorded actions:** for VLA demos, record the clean arm targets and add the offset in the robot-side
   controller with the real gains (see [REAL_ROBOT_AND_VLA.md](REAL_ROBOT_AND_VLA.md)).
3. **Backpack mass:** the sim carries Unitree's 1.9 kg AGX backpack; the real Thunderobot pack is probably heavier — to be weighed.
4. Real-robot measurements (stance at the table, table height, camera, waist control while balancing) — first session in Potsdam.
