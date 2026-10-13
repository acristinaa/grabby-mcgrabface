# From simulation to the real robot, and how to train SmolVLA

## The robot

| Part | Model | In the simulation |
|---|---|---|
| Body | Unitree G1 EDU, 29 DoF | 29-DoF model; the VLA controls 27 values: 14 arm joints, 12 hand motors, waist yaw |
| Hands | BrainCo Revo2 (6 motors each) | Revo2 model, targets 0–1 like the real SDK |
| Policy camera | Unitree G1 head module (2-DoF pan/tilt) with **RealSense D455** | modelled from Unitree's parts (`g1sim/payload.py`); colour camera, 65° vertical FOV, recorded at 320×200 (16:10) — **the only camera the VLA sees** |
| Built-in camera | RealSense D435i | present, not used by the VLA |
| LiDAR | Livox Mid360 | navigation only |
| Onboard computer | NVIDIA Jetson Orin NX 16 GB | — |
| Backpack computer | Thunderobot (Intel Core i7, GeForce RTX) | **modelled as Unitree's 1.9 kg AGX backpack — probably too light** |
| Wrist cameras | none | the sim defines some, the recorder does not store them — keep it that way |

## What must match, or the VLA will not transfer

The VLA learns *"ball at this spot in the head-camera image → these joint targets"*. Anything that shifts the image or the
arm geometry between sim and real breaks that rule.

| Item | Sim now | Action |
|---|---|---|
| Stance at the table (pelvis height, lean, distance) | GR00T's sim crouch: 0.750 m, −2.5°, 5 cm behind the grasp spot | **measure on the real robot** (leg angles + IMU) with the walking controller that will be used; then randomize around it |
| Backpack mass | 1.9 kg | **weigh it**, update `models/unitree_g1_payload/payload.xml`; randomize until then |
| D455 stream | 1280×800 (16:10) → 320×200 | set the real camera to **1280×800**, not 1280×720 |
| D455 tilt during the grasp | 0.85 rad (~49° down) | check the real servo's zero, use the same angle |
| Table height, ball, basket | top 0.74 m, ball r 10.5 cm, basket 28 cm inside | measure the real ones |
| Joint zeros / order | sim | read once from the real robot, compare |
| Arm accuracy under gravity | sim PD gains + gravity offset | record the clean targets; add the offset in the robot driver with the real gains |
| Waist yaw while balancing | pelvis welded in sim | **test first on the robot:** standing, balance on, replay one recorded grasp (arms + waist), no VLA. Unitree's `rt/arm_sdk` includes the waist joints on the 29-DoF G1 |
| Body sway, latency, lighting | partly randomized | randomize |

**Method:** measure → put the numbers into the sim → randomize around them → compare a real head-camera frame with a sim
render from the same pose → add 20–50 real teleoperated episodes.

**Before the robot is available:** record with *wide* stance randomization (pelvis 0.72–0.80 m, lean −4°…+2°, camera tilt
±3°, dock ±5 cm, backpack 1.9–4 kg), so the real values very likely lie inside the training range. Measure in the first hour
with the robot; re-record narrowly if a value lies outside.

## Training SmolVLA

- **One model, one task, one instruction** ("put the volleyball in the basket"). The three variations (plain, distractors,
  after walking) are not three datasets — they are knobs of **one recorder** that randomizes every episode:
  start pose from walk arrivals (crouch), 0–3 distractors (~25 % with none), positions, light, colours, camera, physics.
- **Episodes start at the handover.** The VLA never commands the legs; walking frames would teach it actions nothing executes.
- The fixed-stance data (robot locked at 0.793 m) only checks the pipeline — its camera height is wrong for the real robot.
- **Size:** ~50 episodes to prove the pipeline end to end, then **300–500** for the first real model (~25 s × 30 Hz each,
  ≈ 300k frames). Fine-tune SmolVLA on a cloud GPU (~20–30k steps, a few hours), never on the Mac. Judge it by sim success rate
  on new seeds, not by training loss.
- **Merging datasets** in LeRobot only works if camera keys, image size, state/action size (27), fps and robot type are
  identical. Recording into one dataset from the start avoids it. Old 26-value demos do not fit.

## Before recording at scale
1. Record arm targets **without** the gravity offset (it depends on the sim's gains).
2. Save the handover state with every episode, so evaluation can start from the same states.
3. Distractors and stance randomization in the continuous recorder.
