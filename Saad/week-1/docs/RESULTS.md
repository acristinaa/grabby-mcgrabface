# Results — every test, with numbers

All results are **simulation** (MuJoCo, LeRobot G1 sim, GR00T walking controller). Nothing has run on the real robot yet.
"Pass" always means *all* checks of that script passed (listed per section). JSON files of the main runs are in
[`../results/`](../results/).

---

## 1. Pick and place (fixed robot, `scripts/67_volleyball_pick_place.py`)

The robot stands at the table with the pelvis fixed at 0.793 m. A scripted two-hand "teacher" (`g1sim/ball_grasp.py`, written
by Finnegan Adam) cradles the volleyball on its stand, lifts it, turns the waist from +35° to −35° and drops it into the basket.

**Checks per trial** (first failed = reason): no hand/arm contact with table, stand, basket or a distractor → ball lifted ≥ 6 cm
→ still ≥ 6 cm up after the turn → ball inside the basket for 3 s → on the basket floor → at rest → hands off the ball.

| Date | Version | Run | Result | Notes |
|---|---|---|---|---|
| 10-07/08 | Finnegan's teacher (team repo) | nominal / ±2 cm jitter | 10/10, 50/50 | his numbers |
| 10-08 | **+ distractors** (1–3 of 5 objects on the table) | `--trials 30 --jitter 0.02 --distractors` | **30/30** | nothing touched, no distractor moved (0.0 mm) |
| 10-08 | **+ full randomization** (each group alone) | `--randomize --vary <group>` | 20/20 each | |
| 10-08 | full randomization, all groups together | `--randomize` | 85/100 | every failure: ball 0.53–0.62 m from the pelvis (out of reach) |
| 10-08 | + reach rule (redraw starts outside 0.33–0.52 m) | `--randomize --seed 0` / `--seed 1` | **50/50, 50/50** | |
| 10-09 | merged with Finnegan's new teacher (straight reach, 27-D action with waist) | nominal | 10/10 | |
| 10-09 | same | distractors ±2 cm | 30/30 | |
| 10-09 | same | randomized seed 0 / seed 1 | **46/50**, 50/50 | 4 failures: right wrist touched the basket during the reach (ball still landed) |
| 10-09 | **+ hand lift + gravity offset** (section 5) | nominal / ±2 cm / randomized seed 0 / seed 1 | **10/10, 50/50, 50/50, 50/50** | the 4 basket touches are gone |
| 10-09 | same, run from this repo | nominal / randomized seed 2 (20) | 10/10, 20/20 | check that the repo works on its own |

Randomized = ball + stand offset (±10 cm), robot stand pose (±10 cm, ±10°), physics (mass, friction, motor gains, latency),
image (light, colours, camera pose, noise), 1–3 distractors.

### Waist-limit experiment (10-09) — rejected

Idea: turn the waist less (safer for the real robot's balance?) and let the arms carry the ball the rest of the way.

| Waist limit | Robot straight | GR00T crouch | Max centre-of-mass shift (robot + ball) |
|---|---|---|---|
| none (35°, original) | **pass** | **pass** | **2.3 cm** / **3.0 cm** |
| 25°, 20°, 15°, 10°, 0° | fail | fail | 2.8–3.8 cm |

The arms cannot hug a ball that is 35° off the torso axis (hands 10–54 cm short, they hit the stand), and the centre of mass
moves *more* with stretched arms. Kept as an option (`max_waist_deg`), off by default.

---

## 2. Walking to the table (`scripts/68_walk_to_table.py`)

GR00T walks; our code plans the path and steers. Six situations: table ahead, robot behind the table, table behind the
robot, table beside, obstacle course, anywhere.

**Checks per trial:** no fall → no contact with table or obstacles → arrived in time → within ±10 cm / ±10° of the grasp pose.

| Date | Version | Run | Result | Arrival error |
|---|---|---|---|---|
| 10-08 | first version (side-stepping docker) | 24 trials seed 0 / 24 seed 1 | **24/24, 24/24** | mean ~1 cm, max 3.4 cm, max 2.7° |
| 10-09 | calm path following (smoothed path, speed profile, pure pursuit) | 12 trials | 12/12 | path deviation mean **9.5 → 4.4 cm** |
| 10-09 | checkpoint 0.75 m before the table + straight walk-in | 12 + 12 trials | **12/12, 12/12** | ≤ 4.0 cm, ≤ 2.9°; body ≤ 4° off the table axis (before: up to 87°), no side steps |
| 10-09 | + natural arm swing | 12 trials | 12/12 | ≤ 3.2 cm, ≤ 3.0°; swing ≤ 6°, 0 arm–leg contacts |
| 10-09 | arms down vs. up while walking | same layouts | both 12/12 | path deviation 7.0 vs. 4.1 cm — arms up tracks better |
| 10-09 | after the merge: arms raised to the new grasp start pose | 6 trials seed 7 | **6/6** | ≤ 1.8 cm forward, ≤ 4.3 cm sideways, ≤ 1.6° |

---

## 3. Walk + pick and place in one continuous sim (`scripts/69_walk_pick_place.py`)

GR00T walks the robot to the table, is switched off, the pelvis is held where the robot stopped, the teacher grasps on the
same physics state, then GR00T takes over again and the next trial walks off from there.

**Checks:** the walk checks + ball untouched by the walk (< 1 cm), then the pick-and-place checks.

| Run | Result | What happened |
|---|---|---|
| chain via `67 --arrivals` (12 earlier walk arrivals, separate sims) | 12/12 | first proof that the walk's arrival poses are graspable |
| first single-sim try | 0/1 | the grasp IK needs a geom named `table_top` — renamed |
| second try, robot docked where the grasp scene stands | 0/1 | walk ok, but in GR00T's crouch the hands hit table and stand and knocked the ball off |
| third try, dock 5 cm further back (`CROUCH_BACK`) | **1/1** | first complete walk → grasp → hand-back |
| 6 trials seed 0 (video) | **5/6** | trial 6 docked 2.9 cm sideways → reach knocked the ball off |
| 6 trials seed 1 | 6/6 | |
| 6 trials seed 1, rerun | 4/6 | GR00T is not deterministic; trials 3 and 5 docked ~3 cm sideways |
| **after the robustness fix** (section 5), 12 trials seed 0, video | **12/12** | docked up to **4.7 cm** sideways and all passed; robot never fell after the hand-back |
| same, run from this repo | 1/1 | |

In total before the fix: 15/18 (83 %). After: 12/12 (+1/1). Centre of mass of robot + ball moves 3.6–3.9 cm per grasp.

Video of the 12/12 run: [`media/videos/walk_pick_place_12of12.mp4`](../media/videos/walk_pick_place_12of12.mp4).

---

## 4. How close does the robot have to stop? (crouch window)

After the walk the robot stands in GR00T's crouch: pelvis **0.750 m** (grasp scene: 0.793 m), leaning back **~2.5°**, waist
turned ~3°. Sweep in Finnegan's free-base scene (`volleyball_walk`), 9 starts per distance (±3 cm sideways, ±3°):

| Pelvis behind the grasp scene's spot | 2 cm | 3 cm | **5 cm** | 7 cm | 9 cm |
|---|---|---|---|---|---|
| passed | 3/9 | 6/9 | **9/9** | 6/9 | 3/9 |

→ the walk now stops 5 cm further back (`approach.CROUCH_BACK = 0.05`).

---

## 5. Making the grasp robust to where the walk stops (`scripts/70_handover_sweep.py`)

The handover is replayed **offline** (no walking, no DDS) in GR00T's exact posture, from a grid of dock offsets
(dx = towards the table, dy = sideways, both from the dock point):

| Teacher version | Grid | Passed |
|---|---|---|
| merged teacher | dx −3…+3 cm, dy ±3 cm | **13/25** |
| merged teacher, further back | dx −3…−9 cm, dy ±3 cm | 6/25 |
| + hands rise 6 cm while the waist turns (+ an arched reach, later removed) | dx −6…+3, dy ±4 | 24/35 |
| + **gravity offset** (arched reach removed) | dx −6…+3, dy ±4 | **35/35** |
| same, wider and turned | dx −9…+4.5, dy ±5, yaw ±3° | **36/36** |

Causes found: (1) too close — the hands, turning with the torso, swept through the ball (fingertips 0.98 m, ball top
1.015 m); (2) too far — the stretched arm sagged **7 cm** under its own weight at the end of the reach and the pinky hit the
table (0.748 m fingertip, table 0.74 m).

---

## 6. Tests

`pytest` in this repo: **144 passed** (2026-10-09; with stub `lerobot` / `unitree_sdk2py` packages for import-only tests).
