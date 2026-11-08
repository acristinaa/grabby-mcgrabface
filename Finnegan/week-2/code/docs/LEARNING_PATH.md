# Learning path: from a grasping arm to a robot that does tasks

> **Update 2026-10-07:** the goal is now the user's Level 1–4 ladder (pick and place → vision-based picking → task-level picking → walk and fetch). See §1 "Update 2026-10-07" and `PLAN.md`, SCOPE UPDATE 2026-10-07. The text below (translated from German) is the state of 2026-10-02.
>
> Status as of 2026-10-02 · **Order confirmed by the user (2026-10-02).**
> Goal: in MuJoCo, the G1 gets a task as text ("Bring me the red bottle"), walks there on its own, grasps
> the bottle and brings it back. And Finnegan understands every step.
> **The user's target scenario (2026-10-02):** start a task in the dashboard, like "Go to the next room, put the bottle into
> a box and go back to the other room" – and the robot does it (§10.7).
> How this relates to the other documents: `docs/MASTERPLAN.md` describes the **goal**, `PROGRESS.md` the **status**,
> this document the **path and the learning**. After each step, the logbook (§9) gets a new entry here.

---

## 0. How we learn with every step

Every step (L1, L2 …) runs in the same loop:

1. **Concept first (5 minutes):** Claude explains the idea with a tiny example before any code is written.
2. **Build:** Claude builds it and always says first "Now: …".
3. **Run it yourself:** You start the acceptance check script yourself and look at the video or the browser view.
4. **Three comprehension questions:** They are at the bottom of each step. You answer them in your own words, Claude corrects.
5. **Logbook:** three sentences "What I learned" in §9. Then you commit.

The rule behind it: a step is only done when it **runs and you can explain it.**

**What is always saved here (rule since 2026-10-02):** This document is the complete learning path. Claude adds to it in every
session, without you having to remind it:
- after each step: status and result at the step in §6 (passed, numbers, where the video and images are)
- **every explanation from the chat**, also for side questions, as a section in §10, so you can read it again later
- your answers to the comprehension questions (after correction) and your three sentences in the logbook §9
It is only saved once you commit it. That is why `docs/LEARNING_PATH.md` is in every commit proposal.

---

## 1. Where we are (2026-10-02)

> **Update 2026-10-07 (English, newer than the text below)**
> - **D2 done (code; review gate still open, not committed yet):** the bottle now hangs at ~2.7° instead of ~41° (median, seed 0).
>   `54 --randomize --trials 50`: seed 0 → 25/25 reachable starts (25/50 of all), seed 1 → 35/35 (35/50). 100 % on reachable starts
>   is not a robustness promise (`.planning/reviews/D2-measurement.md`). Evidence: `runs/d2_54_seed0.json`, `runs/d2_54_seed1.json`.
> - **The real G1 has 29 DoF (confirmed 2026-10-07).** The 23-vs-29 question is closed, D5 (mass demo recording) is unblocked (§10.14).
> - **New build order: the goal ladder (Levels 1–3 are the goal, Level 4 is a stretch):**
>   - *Level 1 — pick and place, mastered:* grasp, then drop into a box on a second table. Scripted → demo dataset → trained
>     policy on a rented GPU. ≈ L3a (+ placing) + D4–D7.
>   - *Level 2 — vision-based picking:* the robot recognises object class + colour (detector instead of the red mask). ≈ part of L5.
>   - *Level 3 — task-level picking:* "give me the yellow bottle" → language model picks `pick(object, color)` → state machine →
>     report back to SAP. At one table, no walking. ≈ L4 + L5 + SAP.
>   - *Level 4 (stretch) — walk through rooms, fetch, bring back:* the old masterplan goal. ≈ L3 + L4 + L7.
> - Details and reasons: `PLAN.md`, block "SCOPE UPDATE 2026-10-07".

**Done and measured:**

| What | Result | Where |
|---|---|---|
| G1 simulation with BrainCo hands and the real attachments (backpack, D455 head) | runs | `g1sim/build.py`, `g1sim/payload.py` |
| Walking with a ready-made walking controller (GR00T) | 10 m without a fall, 0.51 m/s | `scripts/50_spike_lerobot_walk.py` |
| Grasping without AI (camera → 3D → IK → grasp) | 50/50 at ±5 cm · with all random variations **90 % with a reachable bottle** (30 of 50 starts were out of reach) | `scripts/54_grasp_scripted.py` |
| Link to π₀.₅ (formats, server, loop) | checked, but no trained model yet | `openpi_g1/`, `scripts/60_openpi_closed_loop.py` |
| Teleop: show demos yourself in the browser | acceptance check 16/16, **not used by you yet** | `scripts/57_teleop.py` |

**The big gap (checked in the code on 2026-10-02):**
- **Walking and grasping live in two separate worlds.** For grasping, the pelvis is bolted down
  (`g1sim/table_scene.py`: the free joint is deleted). The robot cannot fall over there, and a simple PD controller holds the legs.
  For walking, the GR00T controller keeps the balance, and commands go over DDS. No file connects the two.
- **There is no navigation yet, no map, no orchestrator and no text tasks.** Text exists only as one fixed sentence
  for π₀.₅ (`g1sim/openpi_env.py`: "pick up the red bottle").

**Open for you:**
- **The work from 2026-09-22 is not committed yet** (random variation 1.5b, teleop). For the command, see the end of this session.
- Run the tests `scripts/run_tests.sh` (~2–3 min) and try teleop yourself.
- Without code: GPU access (Blocker 3), 23 or 29 DoF on the real G1, Gemma license of the π₀.₅ weights.

---

## 2. Does this work in MuJoCo? Yes, with a division of labor

**MuJoCo is the world:** physics, cameras, LiDAR rays, table, walls, bottle. Everything the robot does happens there.
**Training does not happen "in MuJoCo",** but with data from MuJoCo, on a graphics card (GPU).

| Runs on the Mac (no GPU) | Needs an NVIDIA GPU (rented, like Isaac Lab in the team) |
|---|---|
| Simulation, walking with the ready-made controller, cameras, LiDAR via rays | train π₀.₅ (LoRA: > 22.5 GB of graphics memory) |
| Map, path planning, sequence control, text task (language model via API) | run π₀.₅ (> 8 GB), the sim talks to the server over the network |
| Grasping without AI, recording demos, evaluation | learn walking ourselves with RL (optional, L8) |

Sources (checked on 2026-10-02): openpi README (memory needs), mjlab (macOS only for evaluation), MuJoCo Warp (fast only on NVIDIA).
Current MuJoCo version: 3.14.0 from 2026-09-22.

**What does "training" mean for our goal?** Not everything is learned. Each skill has its own fitting way:

| Skill | How it comes about | Who builds it |
|---|---|---|
| Walking + balance | **learned** (reinforcement learning, trained by NVIDIA, ready-made) | taken over |
| Where to walk | **computed**: map + path search (A*) | we |
| Where am I | first the sim tells us, later **computed** from LiDAR (Fast-LIO) | we / integrated |
| Find the bottle | today a color mask, later a **pre-trained** image model | we plug it in |
| Grasping | today script + IK, later **learned by imitation** (π₀.₅ from our demos) | we |
| Understand the task | a **pre-trained** language model picks skills | we plug it in |
| Sequence (walk first, then grasp …) | **programmed**: state machine | we |

Why not train walking ourselves? It is solved, and our own walking costs weeks of reward tuning
(`PROGRESS.md` → section "WHY RL DOES NOT WORK FOR "TAKE THE BOTTLE"").
If you still want to experience it once: L8.

---

## 3. The big picture: five layers

```
 "Bring me the red bottle"
          │
 ┌────────▼─────────┐  understands the sentence, picks skills            L5   language model (pre-trained)
 │ 1 TASK           │  → Aufgabe(objekt="rote Flasche", ziel="Start")
 └────────┬─────────┘
 ┌────────▼─────────┐  decides what comes NOW:                           L4   state machine (programmed)
 │ 2 ORCHESTRATOR   │  walk · search · align · grasp · carry
 └───┬──────────┬───┘
 ┌───▼────┐ ┌───▼──────┐ Skills:                                         L1-L3, L6, L7
 │3 NAVI- │ │3 GRASP   │ Navigation → (vx, vy, ω)    Grasping → 26-D arm target
 │ GATION │ │          │ every ~50 ms                every 1/30 s
 └───┬────┘ └───┬──────┘
 ┌───▼──────────▼───┐  walking controller (legs + waist, 50 Hz) and      ready-made
 │ 4 CONTROLLERS    │  PD controller per joint (arms, hands)
 └────────┬─────────┘
 ┌────────▼─────────┐  computes forces and contacts (walk world 250 Hz,  MuJoCo
 │                  │  grasp scene 300 Hz), camera images at 30 Hz
 │ 5 PHYSICS        │
 └──────────────────┘
```

Two rules keep the system easy to understand:
- **Each layer talks only to its neighbors.** The language model never moves a joint, it only picks skills.
- **Each joint has exactly one boss.** Legs and waist always belong to the walking controller (joints 0–14), arms and hands to our
  grasping skill (Masterplan §13.1). Two command sources for the same joint are the most common error in such systems.

---

## 4. What already exists and how it works

### 4.1 Grasping without AI (`scripts/54_grasp_scripted.py`)
1. **Image → 3D** (`g1sim/perception.py`): find red pixels (r > 80 and twice as red as green and blue). For each pixel,
   the depth image gives the distance. The pinhole camera model turns this into a 3D point, first relative to the camera, then in the world.
   Result: the center of the cap.
2. **Grasp pose** (`g1sim/grasp.py`): 6 cm below the cap, plus a measured fixed offset between hand and bottle.
3. **IK** (`g1sim/grasp_ik.py`, library `mink`): inverse kinematics answers the question "which joint angles bring the hand
   there?". For this, `mink` solves a small optimization in every step, with hard limits (joint range, at most 3 rad/s,
   distance to the table and the trunk).
4. **Phases** (`g1sim/grasp.py`): look (`hinsehen`) → approach (`anfahren`) → lower (`absenken`) → look again (`neu_hinsehen`) →
   move in (`heranfahren`) → turn the thumb (`daumen_drehen`) → close (`schliessen`) → lift (`anheben`) → hold (`halten`).
   This is a small, hard-wired state machine, the forerunner of the orchestrator.
5. **Success:** the bottle is lifted at least 5 cm, for the whole hold time, without forbidden contacts.

### 4.2 Walking (`scripts/50_spike_lerobot_walk.py`)
1. Our code sends `{"remote.ly": 0.5}`. This is a **virtual** joystick push, not a remote control.
2. The GR00T controller runs two small neural networks 50 times per second (balance and walking) and gives target angles for legs and waist.
3. The commands go over **DDS**. This is a messaging system based on "publish and subscribe", and the real G1
   uses the same. That is why our code stays the same for sim and robot.
4. From this, the sim computes forces and does 250 physics steps per second.
5. Today the command is sent **once**, without feedback. That is why the robot drifts to the side (measured up to 3.4 m over 10 m).
   This is exactly what L1 fixes.

### 4.3 Learning from demos (prepared, not trained yet)
Demo (script or teleop) → `g1sim/episode.py` saves images, state and action as `.npz` → `openpi_g1/convert.py` turns this into
a LeRobot v2.1 dataset → `run.py norm-stats` (mean values for normalization) → training on the GPU → policy server →
`60_openpi_closed_loop.py` lets the sim talk to the server.
The shared language is the **26-D vector** (`g1sim/action_space.py`): 7 joints per arm plus 6 hand motors per hand.

**To do yourself (before L1):** grasp three times with `57_teleop.py --randomize`. Watch `54_grasp_scripted.py --video` and find the
phases again in the video. The commands are in `COMMANDS.md`.

---

## 5. The path: first one thin slice end to end, then make it better

**Recommendation:** first build the whole chain from "text in" to "bottle back" with **simple** building blocks (L1–L5): known position,
script grasp, state machine. Then replace block by block with the learned or more realistic version (L6, L7).

Why this order?
- **No waiting for a GPU:** L1–L5 run completely on the Mac. The GPU (Blocker 3) only slows down L6.
- **See the goal early:** after L5, the sim takes a task as text and does it.
- **Measurable:** every later improvement (π₀.₅ instead of the script) is measured in the full sequence, not only at the table.
- **Good for learning:** you get to know each layer first in its simplest form.

Rejected alternative: train π₀.₅ first (the old order 2.1 → 2.3). That gives the better grasp earlier, but for weeks there is
no full system, and it depends on the GPU, which is still open. The groundwork for L6 (recording demos) can still run on the side at any time. The
camera change to 16:10 already comes in L3a, because otherwise L3 and L4 would have to be measured again.

**What we reuse:** no existing building block is thrown away. Each one is used again or improved in a step.

| Already built | Weakness today | Used again / improved in |
|---|---|---|
| Walking with GR00T (`50`, `g1sim/build.py`, `brainco_env.py`) | command sent blind, drifts away | L1: feedback · L2: path search |
| Image → 3D (`perception.py`) | red mask only | L3a/L3 used further · L5: image model finds objects by name |
| IK (`grasp_ik.py`) | computes with a fixed pelvis | L3: gets the real pelvis and waist pose |
| Grasp phases (`grasp.py`) | first look often blocked, bottle 41° tilted, no placing | L3a: improved · L4: model for the orchestrator |
| Reach (30 of 50 starts too far away) | robot cannot walk there | L3: "align" walks to the right spot |
| Random variation (`randomization.py`) | – | every acceptance check from L3a on |
| Teleop, episodes, openpi link (`57`, `episode.py`, `openpi_g1/`, `60`) | no training yet | L6: record demos → π₀.₅ |
| `TableEnv` interface (`observation()`, `apply(26-D)`) | – | L3: the same interface in the walking world |

| Step | What works at the end | Masterplan | Size | GPU? |
|---|---|---|---|---|
| L0 | Work saved, tests green, teleop tried | – | small | no |
| L1 | "Go to point (x, y)", accurate to 10 cm | 3.2 (part) | small | no |
| L2 | Through the door into the next room, without touching a wall | 3.2 | medium | no |
| L3a | Better grasp at the fixed table: no blocked look, less tilt, placing | 1.4/1.5 | medium | no |
| L3 | Walk to the table, align, grasp while standing freely | 3.4 + new | **large, riskiest step** | no |
| L4 | Full fetch-and-bring sequence, started with a button | 4 (M9) | medium | no |
| L5 | Type a task as text, the sim does it | 1.3 + 4 | medium | no (API) |
| L6 | π₀.₅ grasps instead of the script | 2.1–2.3 (M4–M7) | large | **yes** |
| L7 | Localization with a simulated LiDAR instead of sim knowledge | 3.3 (M8) | large | no (Nav2 is better on Linux) |
| L8 | optional: learn walking ourselves with RL | – | medium | **yes** |

After that comes Stage 5 (real robot). It is not part of this document.

---

## 6. The steps in detail

### L0 — Clean up
- Commit the work from 2026-09-22 (you) · `scripts/run_tests.sh` (you, ~2–3 min) · use teleop three times yourself and say what does not work well.
- **Learning goal:** you know which scripts prove what (`54` grasping, `50` walking, `58` teleop).

### L1 — Go to point: feedback instead of walking blind
- **Goal:** `gehe_zu(x, y, richtung)` (go to x, y, heading) brings the robot to the goal, accurate to 10 cm and 10°. For now, the sim provides the position.
- **Concepts:**
  - *Open vs. closed control loop:* walking off blind (open) drifts away. Measuring all the time and steering again (closed) gets there.
    Example: walking towards a door with closed eyes vs. with open eyes.
  - *P controller:* the farther away the goal, the stronger the command. The turning is also proportional to the angle error.
  - *Coordinate frames:* "world" (fixed in the room) vs. "robot" (moves with the pelvis). vx means "forward as seen by the robot".
  - *Dead zone:* below a command strength of 0.05, GR00T switches to its standing policy (`gr00t_locomotion.py:188-190`). So very small
    commands mean "stand still", and the controller cannot creep up on the goal as slowly as it wants.
- **What we build:** `g1sim/navigation.py` (follow controller), acceptance check `scripts/61_goto_check.py`.
- **Status 2026-10-02: passed.** 10 of 10 goals, on average 6.3 cm and at most 8.7 cm off, angle ≤ 6.3°, no fall.
  Watch: http://127.0.0.1:8020 during the run (`g1sim/liveview.py`). Result: `docs/lernpfad_l1_wege.png`, `docs/videos/lernpfad_l1_goto.mp4`.
- **Acceptance check:** 10 random goals 1–4 m away, all within ±10 cm and ±10°, no fall. Plus a video.
- **Comprehension questions:** Why does the robot drift away without feedback? What happens if the P factor is too large?
  Why must the error be converted from the world frame into the robot frame?
- **Learning source:** PythonRobotics, chapter "Path tracking" (github.com/AtsushiSakai/PythonRobotics).

### L2 — Map and path: into the next room
- **Goal:** a scene with two rooms and one door. The robot finds the way through the door by itself.
- **Concepts:**
  - *Occupancy grid:* the floor is split into 5 cm cells, each one is "free" or "occupied". We make it from
    the walls of the scene.
  - *Inflation:* obstacles are made bigger by the robot radius (30 cm for us). Then it is enough to plan the robot as a point.
  - *Cost map:* cells close to a wall (up to 80 cm) cost more. A* then plans through the middle of the door instead of very close
    past the door edge. A "brushfire" computes the wall distance for each cell: the distance spreads out from the walls like
    a wildfire. Nav2 on the real robot does it the same way ("inscribed radius" + "inflation layer").
  - *A\*:* searches the shortest path through the cells. It always expands the cell with the smallest sum of "path so far"
    and "estimated remaining path". That is why it is fast and still finds the shortest path.
  - *Path tracking (pure pursuit):* the robot aims at a point a bit ahead on the path, like when driving a car.
- **What we build:** two-room scene `g1sim/rooms.py` (room A with the start, a dividing wall with a 1.2 m door, room B with the table; the geometry
  is defined only there and is used for both sim and map), map + cost map + A* in `g1sim/occupancy.py`, path tracking `PathFollower` in
  `navigation.py` (uses the controller from L1), shared sim building block `g1sim/walk_sim.py`, acceptance check `scripts/62_rooms_check.py`.
- **Status 2026-10-02: passed.** 10 of 10 room changes, no wall contact, on average 4.6 cm (max 6.3 cm) off, angle ≤ 8.4°.
  All paths go through the middle of the door, at least 0.6 m from every wall. Map `docs/lernpfad_l2_karte.png`, video
  `docs/videos/lernpfad_l2_rooms.mp4`. In the map you can see: the planned A* path has corners (8 directions), the walked path is
  smooth, because path tracking looks ahead.
- **What went wrong and what we learn from it:** (1) The sim computes in its own thread. Whoever reads its data needs a
  snapshot, or the list changes in the middle of reading. (2) If three image sources render at the same time, the Mac can get stuck in the
  graphics driver, and the process cannot be killed until you log out. That is why there is now only one renderer, and in walking scripts
  the sim renders no camera images. Bonus: because of this, the sim runs at 0.83× instead of 0.50× real time.
- **Acceptance check:** 10/10 room changes without wall contact, goal within ±10 cm (Masterplan T2/T3 with a known position).
- **Comprehension questions:** Why do we inflate obstacles? Why does A* find the shortest path faster than "try everything"?
  What happens if the look-ahead point is too close?
- **Learning source:** PythonRobotics, chapter "Path planning" (A*, grid).

### L3a — Improve the grasp while it is still at the fixed table
- **Why here and not later:** at the fixed table, the sim runs ~20× faster than real time without camera images, so 50 trials take
  ~1 minute. In the walking world (L3) it is only ~0.5× real time. So every improvement is much cheaper to test here. We then take
  the solved errors with us.
- **What we improve (known weaknesses from 1.4/1.5b):**
  - *Head camera at 16:10* like the real D455. This must happen before all further measurements, or everything has to be measured again later.
  - *Blocked first look* (4 of the 5 remaining failures in 1.5b): the hand must be out of the image before the robot looks.
  - *The bottle hangs 41° tilted*, because the fingers are too short for a 6.6 cm diameter: measure a different grasp height or posture,
    do not guess. We set the target value after the first measurement run.
  - *Placing* does not exist yet: stand the bottle upright on the table and let go.
  - Only if the real G1 has 23 DoF: grasping with a locked wrist (today 0/5).
- **Concepts:** error analysis from measured data (which error, how often), regression (an improvement must not break anything else),
  grasp geometry.
- **Acceptance check:** `54 --randomize --trials 50` with a reachable bottle still ≥ 90 %, no more failures from a blocked look,
  tilt measured and smaller, placing ≥ 8/10 upright.
- **Status 2026-10-02 — part 1 passed (D1):** head camera 320 × 200 (16:10) → 91° wide like the D455. New phase `freimachen` (clear the view): hand
  15 cm back, 10 cm outwards, before the robot looks. Same 50 start setups as 1.5b: first look max 13.3 cm → 0.3 cm
  off, reachable bottle 90 % → **95 %**. Part 2 (tilt, placing) follows.
- **Status 2026-10-02 — part 2 (D2) done** (English; code not committed, independent review gate still open):
  - Change in `g1sim/grasp.py`: hand rolled −30° about the finger axis, ring + little finger stay open, lift command
    `LIFT` 0.10 m + `LOAD_SAG` 0.04 m (extra height for the sag under load). Why: §10.10.
  - Tilt: median ~41° → **2.7°** (seed 0) and 2.4° (seed 1), max 4.4°.
  - Success: seed 0 → **25/25** reachable starts (25/50 of all), seed 1 → **35/35** (35/50). On the 20 starts that were reachable
    before, 20/20 (seed 0; seed 1: 31/31). Teleop check `58`: 3/3 episodes, 16/16 checks. New tool: `scripts/63_grasp_sweep.py`.
  - Known side effects: two existing tests fail (`test_ik_erreicht_greifpose[23dof]` — the rolled grasp cannot be reached with a
    locked wrist; `test_greifwert_erst_daumen_dann_finger` — still expects all 6 fingers closed).
  - **Placing is still open → it is now the next step of Level 1, as "drop into a box"** (second table + box, §10.16), not
    "stand upright on the table".
- **Comprehension questions:** Why do we test this at the fixed table and not right away in the walking world? What is a regression?
  Why does the bottle hang tilted?

### L3 — Walking and grasping in one world (the hardest step)
- **Goal:** the robot sees the bottle from 1–2 m away, walks to a grasp position, stops and grasps. While doing this, the
  walking controller keeps the balance.
- **Why it is hard:** until now the pelvis was bolted down. Now the robot stands freely. When the arm moves, the center of mass
  shifts, the pelvis sways by millimeters to centimeters, and the grasp must make up for that. Also, with cameras the sim runs only
  at ~0.5× real time.
- **Concepts:**
  - *Free joint:* the 6-D joint that lets a body stand freely in space and fall. In the grasp scene it is missing on purpose.
  - *Joint authority:* legs and waist belong to the controller, arms and hands to us. The arms go over the same DDS message as
    the legs (`rt/lowcmd`): each side writes only its own joints into it. The hands have their own channels (`rt/brainco/*`).
    So `apply(26-D)` splits the vector: 14 arm values into one message, 12 hand values into the other.
  - *Align (base placement):* the right hand only reaches 0.30–0.46 m in front of the pelvis. So from the bottle position we compute
    a standing spot and walk there with L1.
  - *Same interface:* the new environment gets the same methods as `TableEnv` (`observation()`, `apply(26-D)`). So the
    script grasp, and later π₀.₅, run there without changes.
- **Precondition:** L3a is done, so the camera is at 16:10 and the grasp is better.
- **What we build:** table and bottle in the walking world, a walk-and-grasp environment with the `TableEnv` interface, `ausrichten()` (align).
  Three pitfalls (found in the code by the reviewer on 2026-10-02):
  - The friction settings of the grasp scene (elliptic friction, `impratio` 10) apply to the **whole** world, so also to the
    feet. After taking them over, repeat the 10 m walking test (`50`).
  - The grasp was tuned at a pelvis height of 0.793 m with straight legs and our own waist. GR00T stands with bent knees
    at ~0.75 m and controls the waist itself. So measure the reach again, and give the IK the real pelvis and waist pose in every step.
  - Sim and controller run by the wall clock, and the sim only reaches ~0.5× real time. So the 30 Hz step and "hold for 5 s" must
    follow the **sim time**.
- **Acceptance check:** from 10 random start points ≥ 8/10 grasps (lifted ≥ 5 cm, held for 5 s), no fall. For comparison: at the
  bolted-down table it is 90 % (with a reachable bottle).
- **Risk, honestly:** if the controller sways too much while the arm moves, plan B is to move more slowly while grasping and to
  measure the pose again before closing (`neu_hinsehen` already does this). Plan C is a whole-body controller like GEAR-SONIC (NVIDIA),
  which controls arms and legs together. Our LeRobot version already ships `controllers/sonic_whole_body.py` for this,
  but it has not been tried on the Mac yet.
- **Status 2026-10-08 — part 1 (walk + align) done, part 2 (grasp in the walking world) open.** `scripts/68_walk_to_table.py`
  (`g1sim/approach.py`, `g1sim/approach_world.py`): the table and 0–3 obstacles are put at a random pose around the robot – table
  ahead, robot behind the table (walk all the way round), robot turned away (turn round first), table beside, obstacle course,
  anywhere – then A* + path following (L2) + a new docking controller bring the pelvis to the volleyball grasp pose. **24/24 with
  seed 0**, arrival error ≤ 2.9 cm and ≤ 2.7°, no contact, no fall. The arms ride in `READY_POSE`. Not yet: grasping after the
  walk (the walking pelvis is at ~0.74 m, the grasp was tuned at 0.793 m – the pitfall above), position from localization (L7).
- **Comprehension questions:** Why does the bolted-down robot never fall over? Why must our code not command the leg joints?
  Why do we compute the grasp pose relative to the pelvis and not to the world?

### L4 — The orchestrator: the full fetch-and-bring sequence
- **Goal:** one button press starts: to the next room → search for the bottle → align → grasp → lift → back → place.
- **Concepts:**
  - *State machine:* the robot is always in exactly one state (e.g. `GREIFEN`, grasping). A state ends only through a checked
    condition ("bottle lifted ≥ 5 cm"). Masterplan §13 shows the picture.
  - *Time limit and retry:* each state has a time limit. If it fails, there is a fixed way out, e.g. "align again,
    at most twice", otherwise `FEHLER` (error) with a reason.
  - *Carrying:* the walking controller does not know the arm pose. That is why the robot carries the bottle close to its body (Masterplan T8).
- **What we build:** `g1sim/orchestrator.py`, one skill per state, a sequence log (state, duration, reason).
  **Placing** comes from L3a. If it is not reliable enough yet, the robot first only holds the bottle out.
- **Acceptance check:** 10 runs, ≥ 7 complete. Every failure names the state and the reason.
- **Comprehension questions:** How does the orchestrator know that grasping is done? What happens if the bottle falls while carrying it?
  Why is the sequence logic not in the grasp code?
- **Learning source:** "Behavior Trees in Robotics and AI" (arxiv.org/abs/1709.00084), chapters 1–2. There, the state machine is compared with
  the alternative, the behavior tree.

### L5 — A task as text: "Bring me the red bottle"
- **Goal:** a browser page with a text field (like the teleop page, later in the Paper design "Control Center"). You type a task,
  the orchestrator does it.
- **Concepts:**
  - *Language model with tools (tool calling):* the model gets a list of allowed skills, e.g. `hole(objekt, ziel)` (fetch(object, goal)).
    It does not answer with text, but with exactly one call from that list. It refuses unknown things ("Make coffee").
  - *Why the model never controls joints:* language models can be wrong. The state machine checks every step, the model
    only picks. This is the modern form of "SayCan" and "Code as Policies".
  - *Image model finds objects by name* (Stage 1.3): replaces the red mask, so that "green cup" really finds the cup.
- **Decision at this step (Blocker 4):** which model? Candidates are Gemini Robotics-ER (built for exactly this role, also finds
  objects in the camera image) or Claude via API. Both sit behind **one** function `plane(text) → Aufgabe` (plan(text) → task), so they can be
  swapped. Without a network, a fixed choice menu is the fallback.
- **Acceptance check:** "Bring me the red bottle" → full sequence. "Take the green cup" → does **not** grasp the bottle. "Make me a
  coffee" → polite refusal, the robot does not move.
- **Comprehension questions:** Why does the language model return a function call instead of free text? What happens if it makes up
  an object that does not exist? Which layer notices that?

### L6 — Learn grasping instead of scripting it (π₀.₅, as soon as the GPU is there)
- **Goal:** in the state `GREIFEN` (grasping), π₀.₅ works instead of the script. Success > 60 % at a fixed position and > 85 % with random variation,
  measured against the 90 % of the script (with a reachable bottle).
- **Groundwork that can be done at any time:** record ~100 script demos and 20–30 teleop demos (Stage 2.1). By then the camera is already
  switched to 16:10 (L3a).
- **Concepts:**
  - *Imitation learning:* the model learns from examples "in this situation, you do this", instead of from rewards.
  - *Action chunks:* π₀.₅ plans 30 steps (1 s) at once. Recommendation: only run 10 of them, then ask again.
  - *LoRA:* only small extra matrices are trained, the big model stays frozen. That fits on a rented GPU.
  - *Normalization, distribution shift:* why the model fails when the test situation looks different from the demos.
  - *Language in π₀.₅:* π₀.₅ takes a short command like "pick up the red bottle". Later, the orchestrator provides it.
- **Acceptance check:** Masterplan M4–M7.
- **The path to a really good, checked dataset (status as of 2026-10-02):**

  | # | Step | Done when | GPU? |
  |---|---|---|---|
  | D1 | Head camera like the D455 (16:10) + no blocked first look (= L3a part 1) | same field of view as the real D455; `54` without failures from blocking | no |
  | D2 | Final grasp geometry: measure the tilt and reduce it (= L3a part 2) | tilt measured and smaller, `54` still ≥ 90 % | no |
  | D3 | Decisions before recording (see below) | all four decided and in PLAN.md | no |
  | D4 | Demo factory: script grasp writes episodes, only reachable starts, only successes, all random groups, seeds logged | 2 test demos measured (duration, size), scaled up | no |
  | D5 | Recording: ~100 script demos + 20–30 teleop demos from you with deliberate corrections · *unblocked 2026-10-07 (29 DoF confirmed)* | all pass `episode.validate()` | no |
  | D6 | Build the dataset: `convert.py` → LeRobot v2.1, new `validate_dataset.py` (Masterplan §9.2 at dataset level + units and order against `action_space`), `norm-stats` | validation green, openpi's data loader reads it, `norm_stats.json` exists (= M4) | no |
  | D7 | Quality report: coverage of the start setups, episode lengths, action ranges per joint, **replay test** (play the recorded actions again → it grasps again), watch 5 episodes as video, split training/test 90/10, dataset fact sheet | report without open issues | no |
  | D8 | Final proof: a small test training (first learn a few episodes by heart, then LoRA) = Stage 2.2 / M5 | the model learns the demos | **yes** |

  **Status 2026-10-07 (English):** D1 done · D2 done (see L3a, review gate open) · D3 #1 and #2 decided · D4 probe done
  (`scripts/64_demo_factory.py`: 2 demos, each 615 frames = 20.5 s sim time, ~93 MB, ~16 s compute; 100 demos ≈ 27 min and 9.3 GB,
  `runs/d4_probe.log`) · D5 unblocked, mass recording not started. Proposal, not decided yet: with Level 1, dataset v1 would become pick **and place** (into the box) – see PLAN.md, SCOPE UPDATE 2026-10-07 (g).

  **Status 2026-10-08:** D4 prerequisite for the volleyball scene done: `reset(randomize=True)` there now varies **all** groups (ball +
  stand ±10 cm, robot stand ±10 cm / ±10°, physics, light/colours/camera/noise, 1–3 distractors), starts out of reach are redrawn;
  the teacher passes 100/100 fully randomized starts (`67 --randomize`, seeds 0 and 1). The demo factory does not use it yet.

  **Status 2026-10-10:** the teacher now works in the **walk world** (`volleyball_walk`: free robot in the walking crouch, pelvis
  welded, standing at the stop pose of each layout + a random arrival error): **199/200** random starts (`67 --scene volleyball_walk
  --randomize`), reach map 52/65 with every failure explained. This is the world the v1 demos are recorded in (no-walk mode, §10.44).
  Next: the recorder (D4/Step 5), then 300 demos on the T7 (§10.48).
  **Status 2026-10-14:** D4 done for the volleyball: recorder `scripts/69_volleyball_demo_factory.py` (Step 5), 5/5 test demos on the T7,
  bit-exact replay, independent review GO. The grasp now lifts from the shoulders (user's choice, 200/200 in the walk world). 300 demos
  (Step 6) recording since 17:56. Next: D6/D7 (validate + convert to LeRobot v3.0), then SmolVLA training (Step 7). See §10.49-10.56.

  **D3 — four decisions that shape the dataset (each with a recommendation):**
  1. ✅ *Wrist cameras?* **Decided 2026-10-02: head camera only.** The real G1 has none (confirmed by the user). Episodes,
     dataset and policy only carry `head_image`. π₀.₅'s two wrist inputs get a black image with the mask
     "does not exist" (§10.9).
  2. ⏳ *23 or 29 DoF?* Still open (user unsure, 2026-10-02). If the real G1 has no wrist pitch/yaw, demos with
     these joints cannot be run on the robot, and the grasp with a locked wrist works 0/5 today. Clear this up **before D5**, in one
     of three ways: on the robot (29 DoF = two extra motors between forearm and hand, three waist joints instead of one; when switched off,
     the hand cannot nod at 23 DoF) · order/delivery note or the team/SAP contact · a small SDK read script on the robot
     that shows which joint motors report values. Until then, the masterplan's assumption holds: 29 DoF.
     ✅ **Resolved 2026-10-07: 29 DoF.** The user's robot spec list says "Unitree G1 EDU 29 DoF". The real robot has wrist pitch
     and yaw, so our grasp can run on it. D5 is no longer blocked (§10.14).
  3. *Task:* dataset v1 only "pick up the red bottle" (lift + hold, Masterplan: 5–15 s per episode). Placing and other
     objects come in dataset v2 (with L4/L5).
  4. *Amount:* Masterplan §8.4 asks for 50–100 clean episodes at the start. Our plan: ~100 script + 20–30 teleop, later 10–20 for each
     failure case we find.

  D4–D7 need no GPU. Recommendation: right after L3a, so before L3. Then the grasp is freshly done, and the dataset is ready
  as soon as the GPU comes.
- **Learning source:** LeRobot docs (huggingface.co/docs/lerobot), openpi README.

### L7 — Localization like on the real robot
- **Goal:** the sim no longer tells the position. The robot works it out itself from a simulated LiDAR (Livox Mid360) and the
  IMU (sensor for acceleration and turn rate).
- **Concepts:** LiDAR as a fan of rays (MuJoCo `mj_multiRay`, package `mujoco-lidar`) · SLAM or localization (Fast-LIO) ·
  sensor noise · ROS 2 and Nav2.
- **Note:** ROS 2 and Nav2 now also run on Apple Silicon, but only as community packages. On Linux it is more reliable,
  and the same stack runs later on the real G1.
- **Acceptance check:** Masterplan M8, room change without collisions, goal within ±10 cm, without sim knowledge.
- **Adjust first:** the walls of the L2 scene are only 1 m high. The Mid360 sits on the head (~1.2 m) and, according to the data sheet, sees vertically
  about −7° to +52° (check again before L7). So a head LiDAR would look over the walls → raise the walls to a realistic ~2.5 m.

### L8 — Optional: learn walking ourselves with RL
- **Goal:** experience once how a walking controller comes about. Train a G1 walking policy with `unitree_rl_mjlab` (official from Unitree)
  on the rented GPU, then play it back on the Mac in MuJoCo and compare it with GR00T.
- **Concepts:** reward, PPO, domain randomization, Sim2Sim.
- **Why optional:** not needed for the goal, but the best answer to "how do you teach it to walk?". This fits the Isaac Lab work
  of Cristina and Saad, and the team can use the same GPU.
- **Learning source:** Hugging Face Deep RL Course (huggingface.co/learn/deep-rl-course), mjlab docs.

---

## 7. Terms at a glance

| Term | In one sentence |
|---|---|
| DoF | degree of freedom, one independent axis of motion (elbow = 1, shoulder = 3). |
| PD controller | spring plus damper per joint: pulls to the target angle and brakes swinging. |
| Walking controller | ready-trained network that turns (vx, vy, ω) into steps and balance. Not a remote control. |
| IK | computes the joint angles from "the hand should go here". |
| DDS | messaging system between programs (publish/subscribe), also on the real G1. |
| Free joint | the 6-D joint that lets a body stand freely and fall. |
| Control loop open/closed | without or with constant measuring and correcting. |
| Occupancy grid | floor in cells, each one free or occupied. |
| A* | fast search for the shortest path through a map. |
| State machine | sequence of states with checked transitions. |
| Tool calling | the language model answers with a function call from an allowed list. |
| VLM | vision-language model, finds objects in the image from a description. |
| VLA (π₀.₅) | vision-language-action model, gives joint commands directly. |
| LoRA | cheap extra training: only small add-on parts learn. |
| Domain randomization | vary physics and images at random, so that what is learned becomes robust. |
| Sim2Real | transfer from the simulation to the real robot. |

---

## 8. Learning sources (free)

| Topic | Source | When |
|---|---|---|
| MuJoCo basics | official tutorial notebook, linked on mujoco.readthedocs.io | before L1 |
| Control, path planning, path tracking | github.com/AtsushiSakai/PythonRobotics | L1, L2 |
| Inverse kinematics | *Modern Robotics* (Lynch & Park), ch. 6, hades.mech.northwestern.edu; examples in github.com/kevinzakka/mink | L3 |
| State machine / behavior trees | arxiv.org/abs/1709.00084, ch. 1–2; py_trees docs | L4 |
| Language model as planner | ai.google.dev/gemini-api/docs/robotics-overview | L5 |
| Imitation learning / VLA | huggingface.co/docs/lerobot, openpi README | L6 |
| Reinforcement learning | huggingface.co/learn/deep-rl-course, github.com/mujocolab/mjlab | L8 |

---

## 9. Logbook: what I learned

> After each step, three sentences in your own words. Format: `[date] L<n> — …`

- *(still empty)*

---

- 2026-11-01: v2 sim setup (table with legs, light, textures, height, camera, latency), found and removed sim-only image artefacts, recorded 800 demos (0 failed). Next: run2 training.

## 10. Look it up: explanations from the sessions

> What Claude explained in the chat, in short form. New explanations are added at the bottom, with a date.

### 10.1 What is teleop and how does it work? (2026-10-02)
- **Teleoperation:** a person controls the robot live, like a puppeteer. For us this is **not** how the finished system is
  operated, because the goal stays autonomy. It is **showing how**: every demonstration is saved as a demo, and π₀.₅ learns from
  demos in L6 by imitation.
- **Why in addition to script demos:** the script always grasps the same way and never makes a mistake that it corrects. A person
  sometimes starts off target and corrects. A learned model needs exactly these corrections. Plan: ~100 script demos plus 20–30 from you.
  On the real robot, you do the same with a VR headset (`xr_teleoperate`).
- **Chain per step (30 Hz sim time, `g1sim/teleop.py`):** key/gamepad/iPhone → target for the **hand** (not for single joints)
  → limits (≤ 0.25 m/s, workspace, target at most 5 cm in front of the hand) → IK (the same as for the script grasp) → grip value 0…1
  (first turn the thumb, then close) → 26-D action → physics.
- **Recording:** each row holds "what the robot saw" (3 camera images + joint positions) **before** the action, plus "what it
  did". Success: bottle lifted ≥ 5 cm, without forbidden contacts. Only successful demos go into the dataset.
- **Controls (`scripts/57_teleop.py --randomize`, browser http://127.0.0.1:8010):** W/S, A/D, E/Q = forward/back, left/right,
  up/down · J/L, I/K, U/O = turn, nod, roll · Shift = fine control · G = grasp posture · H = ready pose · Space =
  grasp · Enter = save · ⌫ = discard · 1/2/3 = views. Directions are as seen by the robot.
- **First grasp:** G → move over the bottle and lower the hand (close to it with Shift, the arm lags up to 4 cm behind) → Space,
  wait ~2 s (thumb) → E to lift → Enter. While recording, the sim runs in slow motion, but the recording is still correct.

### 10.2 Reading test results (2026-10-02)
- Start with `scripts/run_tests.sh 2>&1 | tee runs/tests_unit_integration.log`. `tee` also writes the output to a file,
  so Claude can read the result itself.
- Three sections, one per Python environment (project, LeRobot, openpi). One character per test: `.` passed, `F` failed,
  `E` error during setup, `s` skipped. The percentage shows the progress in the section.
- **Green means three times `Exit 0` in the last line.** Skipped sim tests in the first section are normal: they need the
  LeRobot environment and run in the second section.

### 10.3 Watching and reading the L1 result (2026-10-02)
- **Live view** (`g1sim/liveview.py`, http://127.0.0.1:8020): the MuJoCo window freezes on this Mac. That is why the
  script renders ~10 images/s itself and sends them to the browser, like teleop. The current goal is an orange floor marker with an arrow.
- **Top view `docs/lernpfad_l1_wege.png`:**
  - The line **wobbles**, because the pelvis swings sideways with every step. That is walking, not the controller.
  - The paths are **curved**: if the goal is to the side or behind, the robot turns first and then walks.
  - The errors are **just under 10 cm**, because the controller stops when it enters the 10 cm circle, and the robot coasts on.
    More accuracy would only come with a slower approach, and the GR00T dead zone limits that (commands below 0.05 = stand still).
- For comparison: without feedback, the same robot drifted up to 3.4 m over 10 m (script `50`).

### 10.4 L2 in 5 minutes: map, cost map, A*, path tracking (2026-10-02)
1. **Occupancy grid:** floor in 5 cm cells, each one with a wall in it is "occupied".
2. **Inflation:** each wall gets thicker by the robot radius (30 cm). After that, you plan the robot as a point.
3. **Cost map:** cells close to the wall cost more. Result in the test before the first run: all paths from room A to B go
   through the middle of the door (y = 0.98 m for a door from 0.4 to 1.6 m) and keep at least 50 cm from the wall. A* needs
   20–60 ms for this on 92 × 172 cells.
4. **A\*:** always expands the cell with the smallest sum of the cost so far and the estimated rest. Because each cell
   costs at least 1, the estimate never comes out too low, and the path it finds is the cheapest one.
5. **Path tracking:** the robot aims at a point 0.5 m ahead on the path and gives it to the controller from L1 as an intermediate goal.
   On the last 0.5 m, it steers to the real goal with the final heading. This way L2 builds directly on L1.

### 10.5 Is there LiDAR in MuJoCo? (2026-10-02)
- **MuJoCo has no ready-made LiDAR sensor built in, but it has all the building blocks.** A LiDAR uses laser rays all around to measure how
  far away the nearest obstacle is. In the sim this means: "shoot" rays and see where they hit (ray casting).
- Three ways: the sensor `rangefinder` (one ray per mounting point, for single distances) · `mj_ray` / `mj_multiRay` (many
  rays at once, CPU) · the package `mujoco-lidar` with ready-made scan patterns of real devices, also for GPU (not tried).
- **Demo:** `docs/lernpfad_lidar_demo.png`. 720 rays (0.5°) at 50 cm height from the start point cost 1.35 ms. You see room A
  completely, and through the door only a narrow strip of room B. The table is in the "shadow" of the dividing wall. This is exactly why
  localization (L7) needs many scans from different positions, and a map.
- To make it like on the real robot, we still need the scan pattern of the Livox Mid360, noise, and the robot's own motion during a
  scan. On top of that comes the switch from "the sim tells the pose" to "the robot estimates the pose" (Fast-LIO).

### 10.6 Does the robot see by itself and make decisions? (2026-10-02)
- **Status at L2: it does not see anything yet.** Position and map are given, and the test script rolls the dice for the goal. The technical term for this
  deliberate cheating: **privileged information / ground truth**. You use it to solve problems one at a time. If walking works with a
  perfect position but not with an estimated one, the error is in the localization.
- **There are real decisions already:** A* chooses the path (e.g. the middle of the door instead of the edge), path tracking and the controller steer
  again 20 times per second, GR00T decides 50 times per second about every leg joint (learned).
- **So far it can only see by itself when grasping** (Stage 1.4): head camera → red pixels → 3D point with the depth image → look
  again before closing. This is real seeing with feedback, but it only recognizes "something red", not a "bottle".
- **When the helpers go away:** L3 camera while walking, compute the standing spot itself · L4 decide the sequence itself, also when errors happen ·
  L5 recognize objects by name, split a task into steps · L6 arm motion directly from the image (learned) · L7 map and position
  itself from LiDAR. After L7, it sees and decides fully by itself.

### 10.7 After D8, do we have a VLA that can do the whole scenario? (2026-10-02)
- **After D8 there is one building block:** a self-trained VLA (π₀.₅ + LoRA) that controls arm and fingers 30 times per second
  from a camera image and a sentence. But only for **one** skill: grasping and lifting while standing. Only the evaluation (M6/M7) shows how well.
- **The scenario "into the room, bottle into a box, back" needs all building blocks:** the language model plans the steps (L5) →
  the sequence control runs them and catches errors (L4) → navigation + GR00T walk (L1/L2 ✅, L3) → π₀.₅ grasps (D1–D8, L6) →
  π₀.₅ puts it into the box (**new:** box in the scene, demos "put the bottle in the box" = dataset v2) → dashboard with a task field
  (Paper design exists, code with L5).
- **Division of labor instead of one giant model:** a model that learns walking, navigating and grasping at the same time is research at the limit.
  What is reliable is the hierarchy: plan (language model) → run (sequence control) → hand work (VLA) → walking (navigation + GR00T).
- **Success rates multiply:** four steps with 90 % each → only ~66 % for the whole task (0.9⁴). That is why the
  sequence control needs retries, and the whole scenario is what gets measured.
- **Putting it into a box is easier than standing it upright:** the tilt hardly matters. π₀.₅ can learn both skills in one model,
  told apart by the command sentence.
- **Order:** D1–D7 → L3 → L4 (with script grasping and script dropping into the box) → L5 (text task in the dashboard): the scenario
  then already runs without a learned VLA. With a GPU: D8 + L6 replace the script hand, and dataset v2 is added for "into the box".

### 10.8 Aspect ratio, field of view and letterbox (2026-10-02)
- **The real D455** delivers 16:10 (1280 × 800) and sees 90° wide, 65° high (Intel data sheet, written in `payload.xml`).
- **The sim rendered square images** (224 × 224) with 65° height, so also only 65° width: 12° less than real on the left and on the right.
  Now 320 × 200 (16:10) → 91° wide. Comparison image: `docs/lernpfad_l3a_kamera.png`.
- **Why it matters:** π₀.₅ learns where things are in the image and how big they look. If the real image looks wider, everything is at
  different pixels and looks smaller, and what was learned does not fit. Technical term: **Sim2Real gap**.
- **Letterbox:** π₀.₅ needs 224 × 224. So openpi itself shrinks the image to 224 × 140 and fills the top and bottom with black
  (`ResizeImages` = `resize_with_pad`). Nothing is cut off, and sim and real robot go through the same conversion.
  The price: in the model image, the bottle is only ~20 pixels wide. The training (D8) will show whether that is enough.
- **Segmentation image:** MuJoCo can tell for each pixel which body is seen there. With this, we measured that the
  head camera sees ~5500 pixels of its own right hand in the ready pose. This was the cause of the blocked first look.

### 10.9 Only one camera, but π₀.₅ has three image inputs (2026-10-02)
- π₀.₅ has fixed slots for three images: head (`base_0_rgb`), left and right wrist. If a camera is missing, its slot gets a
  black image plus an **image mask** "does not exist". The model then ignores the slot. openpi itself does it this way
  (`libero_policy.py`).
- Why not just use the sim wrist cameras? Then π₀.₅ would learn to rely on images that the real G1 never delivers.
  On the robot, it would miss exactly the information it learned with. Rule: **the sim only records what the real robot also has.**
- The sim wrist camera stays as a help for the person on the teleop page ("display only") and for the check images in `54`.


### 10.10 Why does the bottle hang tilted? First guessed, then measured (2026-10-02)
- **Measured** (one trial, fixed start setup): tilt 0° until closing · 11° after closing · 27° after lifting · 40° after
  holding for 5 s. At the end, only three spots touch the bottle: the thumb tip at the back, index and middle fingertip at the front. Ring finger, little
  finger and palm do not touch it (the same in all 19 passed random trials).
- **Pinch grasp vs. power grasp:** in a pinch grasp, fingertips hold the object at a few points. In a power grasp, the
  fingers and the palm wrap around it.
- **First guess (Claude, before the measurement series) — proven wrong:** "The three contacts form a hinge, the center of mass hangs beside it
  with a lever arm, so grasping lower helps." Measured: grasping lower (7.5–12 cm below the cap) makes it **worse** (44–50°).
- **Actual cause (measured, agent D):** the motor that turns the thumb in front of the palm has only **0.5 Nm** in the BrainCo model,
  each finger 2 Nm. Ring finger and little finger push against the bottle from below and **turn the weak thumb back** (thumb
  1.0 → 0.70). The bottle tips with it, until a balance sets in at ~40°. Technical term **back-driving:** an outside
  force is stronger than the motor and turns it back against its will.
- **The fix (D2):** ring finger and little finger stay open → the thumb stays at its end stop → 7°. In addition, roll the hand by 30°
  about the finger axis (the palm looks a bit downwards) → **2–3°**. More contacts do not help automatically: the power grasp with the
  palm (agent B) stayed at 16°.
- **Lesson:** a plausible explanation is not yet a cause. Only the measurement (thumb angle over time) showed what really happens.
- **Why the tilt is a problem:** (1) Standing it upright is impossible at 40°: the bottle touches down on its edge and falls.
  (2) When carrying it while walking, a three-fingertip grasp rocks, and the bottle slips out more easily. (3) On the real robot, a
  tight grasp fails first, because real friction is different from the sim (and an open bottle would leak). (4) π₀.₅ would learn the tilted grasp
  from the demos too. Not solved by D2: starts out of reach (→ L3 "align"), the placing itself, 23/29 DoF.
- **Four levers that D2 measures:** A grasp height towards the center of mass · B deeper into the hand (power grasp) · C turn the hand so that thumb and
  fingers are exactly opposite each other · D lift more slowly or close differently.
- **Parameter study:** run the same task with many settings on the **same** start setups and compare. Tool:
  `scripts/63_grasp_sweep.py`. The constants in `grasp.py` only change once a measurement run proves a better posture.
- **Median:** the middle value when you sort all trials. A single outlier hardly moves it, but it does move the average.
- **The contact model stays as it is:** tweaking the friction to look good would keep the bottle straight in the sim, but not on the real robot.
  That would be an artificial Sim2Real gap.

### 10.11 Do the BrainCo hands decide whether the G1 has 23 or 29 DoF? (2026-10-02)
- **No.** The Revo2 hand always has its 6 motors. 23 or 29 DoF is about the **G1 body**: with 29 DoF there are two
  extra motors in the forearm that let the wrist nod (pitch) and turn sideways (yaw), and the waist has three joints instead of one.
- **10-second test:** can the hand nod up and down at the wrist, powered and not just loose? Yes → 29 DoF.
  Other ways: delivery note/order (model "G1 EDU" with its equipment variant) or an SDK read script that shows which joint motors
  report values.
- **Why it matters:** our script grasp uses both wrist joints. With a locked wrist it works 0/5 today. Demos with
  joints that the real robot does not have could not be run on the robot. So clear this up before D5; it does not block D2 and D4.

### 10.12 Are we training right now? Parameter study vs. training vs. dataset (2026-10-02)
- **Parameter study (D2):** the hard-coded script grasp runs many times with a slightly different hand posture (grasp height, bottle deeper
  in the hand, hand rotation, speed). We count success and tilt on the same 50 start setups. The person (or Claude) then picks the
  best setting and writes it into `grasp.py`. Nothing learns by itself. The Mac gets loud, because 4 agents compute in parallel.
- **Training (D8):** a neural network (π₀.₅) adjusts millions of internal weights by itself, by learning from examples. This only
  happens on a rented Linux GPU, never on the Mac.
- **Dataset (D4–D7):** the demo factory records the improved script grasp as episodes (camera image + joint angles + action every
  1/30 s). π₀.₅ learns from them later.
- **Why D2 first:** π₀.₅ copies what the demos show. A tilted grasp in the demos would be learned too. In the script, an
  improvement is cheap (50 trials ≈ 1 minute); in the learned model, only through new data and new training.

### 10.13 Why big AI models can't run on the robot, and what runs where (2026-10-06)
- **Big open models are too big.** The large open vision-language models need roughly 200–800 GB of GPU memory. The robot's
  computers have far less (Jetson Orin NX: 16 GB; the Thunderobot backpack PC has an RTX card, model and memory still unknown).
  So big models run **in the cloud**, for example Claude via AWS Bedrock (pay per call, about $3–6 per 1,000 "image + decide" calls).
- **Speed decides the place.** Motor commands go out 500 times per second, the walking policy runs 50 times per second. A cloud
  round trip takes far too long for that, and a dropped connection would make the robot fall. So **walking and balance always run on
  the robot.**
- **Cloud is fine for "thinking":** understanding a task or planning the next step can take a second or two.
- **Laptop (Mac):** simulation, our code, small models. **Rented GPU** (e.g. AWS Frankfurt g6e.xlarge, L40S 48 GB, $2.327/h):
  training π₀.₅ with LoRA (needs > 22.5 GB). Running π₀.₅ needs > 8 GB, so whether it can run on the robot depends on the unknown RTX card.

### 10.14 What 23 vs 29 DoF means and why it mattered (2026-10-07)
- **DoF (degree of freedom)** = one joint that a motor can turn on its own.
- **23 DoF:** each arm has shoulder (3) + elbow (1) + wrist **roll only** (1) = 5 joints; the waist has **1** joint.
  **29 DoF:** the wrist can also **pitch** (nod up/down) and **yaw** (turn sideways) = 7 joints per arm; the waist has **3** joints.
  Legs are the same in both (6 per leg). 12 + 1 + 2 × 5 = 23 and 12 + 3 + 2 × 7 = 29.
- **Why it mattered:** our scripted grasp uses wrist pitch and yaw. With a locked wrist it worked 0/5 times, and after D2 the rolled
  grasp is ~4 cm off with a locked wrist. Demos that use joints the real robot does not have could never be replayed on it.
- **Now resolved:** the robot is a G1 EDU with 29 DoF. So D5 (mass demo recording) can start. The failing 23-DoF test still needs a fix
  or a clear "expected to fail" mark.

### 10.15 Open source is not one thing: permissive vs copyleft (2026-10-07)
- **Permissive licenses (Apache-2.0, MIT):** you may use, change and sell the code, also inside closed company software. You only keep
  the license notice. Examples: openpi (Apache-2.0), RF-DETR sizes N/S/M/L (Apache-2.0; the other sizes use a different license, PML-1.0).
- **Copyleft licenses (GPL, AGPL):** if you share a program built with it, you must publish **your whole program** under the same
  license. **AGPL** adds a **network clause**: even letting people use it over a network (a web service) counts as sharing.
- **Ultralytics YOLO is AGPL-3.0**, and Ultralytics says this covers the code, the pretrained weights **and models you train yourself**
  with it. For an SAP project that means: explicit approval, or a paid Enterprise License.
- **Recommended:** RF-DETR (Apache-2.0) for Level 2. Avoid YOLO-World (GPL-3.0), YOLOE/YOLOv10 (AGPL) and DEIMv2 (non-commercial only).
- Model weights can have their own terms: π₀.₅ weights follow the Gemma terms, not openpi's Apache license.

### 10.16 Why a box instead of placing on a flat table (2026-10-07)
- **Placing on a flat table is hard:** a ball rolls away, and a bottle must touch down exactly upright at the right height, or it tips
  over. That needs slow lowering, contact sensing and a careful release.
- **A box is forgiving:** its walls catch small position errors. The bottle may land a few cm off or a bit tilted and still count.
- **Simple success test:** "the object is inside the box and has been still for 3 s".
- **Real example (Reddit, 2025-11-05):** a G1 at a fixed table sorts cans by colour by dropping them into two boxes. Their lessons:
  boxes are forgiving, a narrow moving camera view works better than a wide one, tape on objects helps against glare.

### 10.17 Critical path: why three people don't make it three times faster (2026-10-07)
- **Critical path** = the longest chain of steps where each step needs the one before. The project cannot finish faster than this chain.
- Example: placing needs the second table and the box; demos of pick-and-place need placing; training needs the demos.
  A second person cannot build "placing" while the box does not exist yet.
- **Extra people help on side tracks** that do not wait for the chain: Level 2 (photos, labels, detector), the SAP stand-in page,
  Docker. That is why these are good teammate tasks.
- Extra people also cost time: talking, agreeing on interfaces, merging work. So split work by **clear interfaces** (e.g. the report JSON).

### 10.18 Docker in plain words, and what it can't do on a Mac (2026-10-07)
- **Docker** packs a program together with all its libraries and settings into a **container**. The container runs the same on every
  computer, so a teammate can start our sim with one command (`docker compose up`) instead of a day of setup.
- **Image** = the packed, ready-to-start box. **Compose** = a file that starts several containers together (e.g. sim + teleop).
- **Limits on the Mac:** Docker runs inside a small Linux virtual machine. So: **no GPU** in the container (MuJoCo renders on the CPU,
  `MUJOCO_GL=egl/osmesa`), **no `mjpython` window** (use the browser live view), **no DDS multicast to the real robot** (the VM's network
  blocks it). Daily development and real-robot work stay native on the Mac.
- **Licensing:** Docker Desktop's terms (updated 2026-08-26) require a paid plan for shared university-group projects and big companies.
  So on the Mac we use **Colima** (free, MIT license), on Linux plain Docker Engine.
- Plan: Stage 0 (lock environments + 3 small code fixes), Stage 1 (Dockerfile + compose), Stage 2 (openpi's Docker on a GPU). Waiting for
  the user's go.

### 10.19 Imitation learning: how much data is really needed (2026-10-07)
- **Imitation learning** = the model copies demonstrations: "in this situation, do this".
- **Real-world reference (Reddit can sorting, 2025-11-05):** ACT (Action Chunking with Transformers, a small imitation model) needed
  **~3 hours of real demos** for **one narrow task** (right arm only, fixed table, cans into two boxes). No success rate was published.
- **Sim demos:** cheap. The demo factory makes one in ~16 s of compute, so hundreds are easy. But they show sim physics and sim images,
  so a model trained only on them often fails on the real robot (sim-to-real gap).
- **Real teleop demos:** slow and need people, a VR headset and robot time. But they show the real cameras, real friction, real mistakes.
- **Our plan:** sim demos first, to make the whole pipeline work and reach Level 1 in sim. For the real robot, plan **hours** of real
  teleop demos, not dozens.

### 10.20 Teacher vs. VLA: why there is no intelligence yet (2026-10-07)
- The **teacher** (`g1sim/ball_grasp.py`) is programmed: every step is a rule. It only exists in the sim to **record demonstrations**.
- The **VLA** (SmolVLA or π₀.₅) learns by copying those recordings: camera image in, movements out. On the real robot only the VLA runs.
- Order: teacher works (done, 50/50) → record demos (= the dataset) → train → the VLA does it alone.

### 10.21 Training vs. running a model (2026-10-07)
- **Training** happens once per dataset, takes hours, and lag does not matter, so a rented cloud GPU is fine.
- **Running** happens live while the robot moves; every delay makes it react late. So the trained model runs on the robot (backpack), not in the cloud.
- Training cost depends on the number of training steps, not directly on the number of demos (π₀.₅ LoRA ≈ 15 H100-hours ≈ $30–60 per 30k steps).

### 10.22 SLAM, navigation and BotBrain (2026-10-07)
- **SLAM** = the robot builds a map and knows where it is in it. **Navigation** = plan a path to a goal and walk it without hitting things.
- **BotBrain** contains both (RTAB-Map + Nav2) plus a G1 driver, missions and a web dashboard. We use only the local parts; the dashboard
  needs a cloud login (Supabase), so it stays off.
- Walking itself (steps, balance) is Unitree's built-in controller; navigation only sends "walk this fast in this direction".


### 10.23 Recording vs. dataset vs. trained model (2026-10-08)
- A teacher run that only prints "success yes/no" produces **no training data**. Data exists only when a **recorder** saves every control
  tick: head image + joint angles (input) and the commanded action (output).
- Order: teacher works → record many demos (= the **dataset**) → convert + validate → **train** the VLA → test the VLA alone.
- On 2026-10-08 no volleyball dataset existed yet (only 2 bottle test recordings); no VLA was trained.

### 10.24 Variation (domain randomization): why the VLA needs it (2026-10-08)
- The VLA does not memorise the variants. It learns **what stays the same** across all demos (ball, basket, task) and to **ignore** what changes.
- Same ball spot in every demo → the VLA learns one fixed movement and ignores the camera. Ball, basket and robot stance change → it must look.
- Most useful for the real robot: what the ball and room look like (our real ball is **grey**), table height, camera angle, distractors
  (other objects), recovery after small disturbances, delay. Add one group at a time; more variation needs more demos.

### 10.25 Walk-up: script walks, VLA grasps, robot "welded" while grasping (2026-10-08)
- Same split as on the real robot: navigation (BotBrain there, a script in sim) owns the **legs**; the VLA owns **arms, hands and waist yaw**.
- In sim, GR00T walks the robot to a stop pose in front of the table. At the **handover** a **weld** (a sim constraint that pins the pelvis
  to a fixed point, like an invisible harness) holds the robot, and the demo recording starts. On the real robot Unitree's controller balances.
- The robot never stops exactly at the goal; that imprecision is useful variation for the VLA.
- Found by measurement: GR00T stands in a slight crouch, so the **knees hit the table** if the pelvis is closer than ~0.205 m to the edge,
  and the arms must be raised **before** walking up (raising at the table sweeps the fingers through the top).

### 10.26 Finding the ball by shape, not colour (2026-10-08)
- The depth camera gives a 3D point per pixel. Points above the table are grouped into clusters, and a sphere with the **known ball radius**
  is fitted into each. The ball is the cluster that fits a sphere best; a cylinder or a box fits badly and is rejected.
- Why: the real ball is grey, and the red-colour check would also have picked up a red cylinder distractor.
- The basket is found by its **rim** (points at rim height that form a square of the basket's size).

### 10.27 Regression tests: why the bottle tests ran again (2026-10-08)
- A **regression test** checks that what already worked still works after a change.
- Going from 26 to 27 values changed a file every path shares (`action_space.py`), so the bottle demo factory was used as the only complete
  record → validate → convert chain. One demo, ~16 s. Result: 27 values recorded, 0 validation errors, converted with `kWaistYaw`.

### 10.28 Storage vs. compute: what the T7 SSD helps with (2026-10-08)
- An external SSD gives **space** (300 demos ≈ 25-30 GB) and makes it easy to **move data** to a GPU machine. It does **not** make training faster:
  training load is on the chip (MPS) and memory. Only a real GPU (backpack RTX, AWS, rented) takes training off the laptop.
- exFAT so Linux can read it too. Samsung T7 = main drive; SanDisk Extreme PRO = backup (the V2 model had a data-loss firmware bug in 2023).

### 10.29 Working together on one repo with branches (2026-10-08)
- `main` always works. Each person works on an own branch and merges back through a **pull request** (a request on GitHub to merge, which
  the other person can look at first).
- Conflicts come from two people editing the **same lines of the same file**, so split work by files, not only by topic.
- Merge `main` into your branch at least daily; run `scripts/run_tests.sh` before every merge. Only one person edits `PROGRESS.md`.

### 10.30 Making the grasp look natural: measure, don't eyeball (2026-10-08)
- The VLA copies the teacher's movements exactly, so how the teacher moves matters before any demo is recorded.
- New start/end pose found with **IK** (inverse kinematics: give the hand position, the computer finds the joint angles), then checked by
  **forward kinematics** (joint angles in, positions out): clearance to ball/table/basket, how far the elbows stick out, camera view.
- Lesson: the first natural pose knocked the ball off its stand in 13 of 50 runs, because turning the waist with the hands low swept the
  fingertips through the ball. Fix: raise the hands, turn, reach from above; the reverse at the end. Back to 50/50.
- The G1's elbow can only bend to about 60° (-1.047 rad), so some human poses (hands high and close to the chest) are impossible for it.
- Round 2 (same day): a **grid search** tried hundreds of joint combinations and kept only those with the elbows beside the body,
  palms facing in and ≥ 4 cm to everything. The reach became one **straight line** with a **cosine speed profile** (start and stop at zero
  speed, like a human arm), instead of "above the ball, then down". Grasp success stayed 50/50.

### 10.31 Why distractors on the table (2026-10-08)
- If the ball is the only thing on the table, the VLA can learn "grab whatever is there" or "grab the red thing" and still get every
  sim demo right. On a real table there are cups, cans and boxes, and that shortcut fails.
- So each demo gets 1–3 other objects, including a **red cylinder** the same colour as the ball: the model has to use shape, not
  just colour. That is domain randomization for *what else is in the scene*, not just for poses and light.
- MuJoCo cannot add or remove objects between episodes without rebuilding the model, so all distractors are always in the model;
  the unused ones are parked far away on the floor where no camera sees them.
- They are placed only where the scripted teacher cannot hit them; a demo where the arm knocks something over would teach bad habits,
  so touching a distractor counts as a failed demo.

### 10.32 What makes a VLA dataset varied enough (2026-10-08)
- A VLA copies what it sees in the demos. If the ball is always in the same spot, it learns one fixed arm movement and never learns
  to *look* where the ball is. Variety in the data is what forces it to use the camera image.
- Order of value: (1) the existing randomization switched on in the scene we record (positions, robot stand, physics, light,
  camera); (2) variety in *how* the teacher moves, above all **recovery demos** (start slightly wrong, then correct); (3) several
  targets and several phrasings of the instruction (Level 2/3: otherwise the model ignores the text); (4) visual variety (floor,
  walls, textures); (5) distractors in more places.
- Keep test seeds and some never-seen variation apart, so you measure generalization and not memory.
- Rough sizes: one task ~200–500 good demos; several targets and instructions ~1–2k. Sim alone rarely transfers: plan 50–100 real
  teleop demos on top.

### 10.33 Why the teacher must know where the robot really stands (2026-10-08)
- Randomizing the robot's stand pose moves the whole robot. The old teacher turned the waist to a fixed +35° / −35° in world terms,
  so a robot turned by 8° would reach 8° past the ball. Now it measures the pelvis pose and turns by as much more or less as the ball
  and the basket moved around its waist axis. On the nominal start that is still exactly ±35° (the runs are bit-identical).
- Each random group alone was fine (20/20), but together 15 of 100 failed: a stand 10 cm back plus a ball 10 cm further out puts the
  ball 0.6 m away, out of reach of a two-hand lift. Ranges that are safe alone can add up. Fix: `reset` redraws starts where the ball
  is not 0.33–0.52 m from the pelvis – the same idea as "only reachable starts" in D4. Then 100/100.
- Lesson: test every random group alone **and** all together; a failure that only shows up in the combination is normal.

### 10.34 Move the world, not the robot (2026-10-08)
- To start the robot "behind the table" or "turned away" you could teleport the robot. A walking robot does not like that: the
  walking controller has a memory of the last steps, and a jump confuses it. Instead the table and obstacles are *mocap bodies* –
  furniture that collides but can be put anywhere between two trials. The table is placed relative to wherever the robot stands now.
- What matters for the robot is only the *relative* situation (where is the table from my point of view, and which side is the work
  side). Moving the table gives every relative situation without touching LeRobot's sim.
- Bonus: trials chain. Trial 5 starts exactly where trial 4 ended – like a real robot that goes from one job to the next.

### 10.35 Why the robot stopped 5 cm short: a dead band (2026-10-08)
- A *dead band* is a range of small commands that produce (almost) no movement. Measured: GR00T walks ~0.045 m/s slower than
  commanded; at 0.05 m/s it hardly moves, and after a slow stop it settles ~2 cm back.
- A plain P controller ("speed = gain × distance") commands smaller and smaller speeds near the goal – exactly the ones inside the
  dead band. So the robot stepped on the spot 4–6 cm short for 30 s.
- Fix in `approach.Docker`: add the dead band in the direction of travel while still clearly off, aim 2 cm deeper (for the settle-back),
  average the pelvis over ~2 steps (it sways ±2 cm with every step), stop, stand 1 s, measure, and dock again if > 4 cm / 4° off.
- Lesson: measure the actuator first (here: command vs. real speed), then design the controller for it.

### 10.36 Watching changed the result: control rate and rendering (2026-10-08)
- With the live view and the lab view both drawing pictures in the same loop as the controller, the controller ran at ~8 Hz instead of
  20 Hz. A filter that averaged "the last 12 samples" then averaged 1.6 s instead of 0.6 s. The delay made the robot overshoot
  ±20 cm back and forth: 2/12 passed.
- Fix: the controller counts in *sim seconds*, not in loop passes, and the lab view reuses the live view's picture instead of drawing a
  second one (~14 Hz). Then 24/24 with both viewers open.
- Rule: a controller must not depend on how fast the loop around it runs. Measure the real rate, and use timestamps.

### 10.37 Two bugs that were not ours to begin with (2026-10-08)
- **Bus error in the live view:** MuJoCo's `mjv_connector` only fills shape and position of a scene object; the rest must be set by
  `mjv_initGeom` first. The arrow marker skipped that, so it carried random memory (material and texture numbers) into the graphics
  driver – the process died. L1/L2 had just been lucky. Now initialized, and a test fills the slot with garbage first.
- **Hands on the table:** at LeRobot's zero arm pose the hands point forward only ~6 cm above the table top at walking height (the
  walking robot crouches to ~0.74 m). Walking with the arms in `READY_POSE` gives 13 cm and is also the pose the grasp starts from.

### 10.38 Why the walk looked funky, and why "slower" is only part of the fix (2026-10-09)
- The walking controller (GR00T) decides where each foot goes; we only send "this fast, this direction, this much turning". So slowing
  down does not make the steps more decisive – and below ~0.1 m/s GR00T shuffles on the spot. It walks most cleanly at a steady speed.
- The wobble came from *our* commands: the A* path zig-zags on its 5 cm grid, the old follower turned hard, then corrected, and stopped
  to turn at every bend. Fix: pull the path taut, round the corners (*Chaikin*: cut every corner at 1/4 and 3/4, repeat), plan a
  *speed profile* (fast on straights, slow in curves, gentle start and stop), and follow with *pure pursuit* (steer along the arc to a
  point a little ahead on the path, walking and turning at the same time).
- A surprise found by measuring, not by guessing: turning on the spot, GR00T drifts ~5 cm/s sideways. A slower turn drifted more (35 cm).
  Now the follower steps back to where the turn started while it turns.
- Result: average distance to the path 9.5 → 4.4 cm. Lesson: first measure the problem (path deviation), then change one thing at a time
  and compare on the same layouts (seed).

### 10.39 The checkpoint: align away from the table, then walk straight in (2026-10-09)
- Before: one controller did the last 0.4 m with side steps and turning at the same time. When the robot came in at an angle it
  shuffled sideways with its body turned up to 87° away from the table ("penguin walk") – messy, and right next to the table.
- Now, like a person: walk naturally to a spot ~3–4 steps before the table, line up there (small side steps are fine, nothing is
  close), stand, raise the arms, then walk straight in – forward steps only. The body stays within 4° of the table axis.
- Why the arms go up at the checkpoint and not at the table: raising them right in front of the table could swing the hands into the
  table edge; standing 0.75 m back there is room.
- A *lead-in*: the planned path joins the line into the table 0.7 m before the checkpoint, so the robot already arrives facing the
  table instead of making a sharp last-second turn.
- Measured surprise: arms hanging down did not make the walk steadier in sim – GR00T followed the path worse (7.0 vs 4.1 cm). A
  learned walking controller can react to an arm pose it rarely saw in training; on the real G1 (Unitree's own controller) this may
  differ. Lesson: test a "this should help" idea alone, on the same layouts.


### 10.40 "Bolted pelvis" and the waist risk when walking and grasping are merged (2026-10-09)
- In the grasp scene (`g1sim/table_scene.py`) the robot's *free joint* is deleted and the pelvis is fixed at 0.793 m – like a robot on
  an invisible pole through its hips. It cannot fall or sway, legs and waist only hold their angles, and the arm IK can assume the
  shoulders never move. That is why the ball teacher reached 100/100 there: the ideal case.
- In the walking world the pelvis is free: GR00T balances, the pelvis sways 1–2 cm per step and sits lower (~0.74 m), and arm
  movements with the 0.27 kg ball shift the centre of mass – the hands must follow the real body pose.
- The ball teacher turns the **waist** +35° (to the ball) and −35° (to the basket). In the walking sim GR00T owns 15 joints: 12 leg
  joints **and** the 3 waist joints, with no "waist angle" input (only walk speed, turn speed, body height 0.5–1.0 m). Options:
  (1) override GR00T's waist target – may upset its balance, needs a test; (2) turn the whole body with the feet – GR00T drifts
  ~35 cm in a 180° turn on the spot; (3) change the task so no big waist turn is needed – the existing teacher would be rebuilt.
- Plan: Stage 0 tests option 1 first (stand at the table, raise to grasp height, waist ±35°, then with arm motion) before anything
  is built on it. The real G1's own controller may treat the waist differently – the real robot needs its own waist test.

### 10.41 Tests vs. review: what each one catches (2026-10-09)
- **Unit tests** check small building blocks alone; **integration tests** check several parts together in the sim; **acceptance runs**
  repeat the real task many times (e.g. 100 random grasps). All three only check what someone thought of in advance.
- A **review** (someone else reads the code) finds what nobody thought of. Example from the merge: Saad's walk script mixed 26- and 27-value
  vectors - every test was green, it was found only by reading.
- Rule we use: a review where a hidden error is expensive (code that produces the training data), not for docs or small UI changes.

### 10.42 Merging two people's work (2026-10-09)
- Git merges lines automatically; a **conflict** is where both people changed the same lines and a human has to decide.
- Even files without a conflict can be wrong together (here: Saad's arm poses were 26 values long after our change to 27).
- So after a merge: run the acceptance runs of **both** sides. They found the sagging wrist on the basket rim (46/50 → fixed → 50/50).

### 10.43 What a demo contains, and why no walking is in it (2026-10-09)
- The VLA only learns **arms, hands and waist** (27 numbers). Walking stays scripted (BotBrain on the real robot). So a demo starts at the
  **handover**: the robot already stands at the table. One demo = turn to the ball → reach → grip → lift → turn → drop into the basket →
  arms back → face front, ~30 s, with the prompt "put the volleyball in the basket".
- Every demo is different on purpose: ball and basket position, where the robot stands, physics, light, colours, camera noise. Otherwise
  the model memorizes one motion instead of looking at the image.

### 10.44 No-walk mode: why the demos don't wait for the walk-up (2026-10-09)
- The walk-up only decides **where the robot stands** at the handover. We can draw that position at random (stop pose + a small
  arrival error) and record right away. Later, the real walk-up must just deliver positions inside the same range.
- But record in the **right world**: the fixed scene stands 4.3 cm higher (pelvis 0.793 vs 0.7505 m in the walking crouch) and closer to
  the table than the knee allows. A VLA trained there would see a view it never sees after walking or on the real robot.

### 10.45 The depth window: knee vs. arm reach (2026-10-10)
- The G1's hips are at table height, so the **knee** touches the table if the pelvis is closer than ~0.205 m to the edge. We stop at
  0.458 m from ball and basket, which leaves 4 cm (2 cm arrival error + 2 cm margin).
- The **arms** lift the ball reliably up to ~0.53 m away; at 0.547 m the lift sags. So the robot has only ~5-6 cm of depth to be wrong
  by. Measured with a **reach map**: a fixed grid of stances (back, sideways, turned) instead of random tries, so you see the borders.
- Surprise: moving ball and basket **deeper** onto the table pulls the robot **closer** to the edge, because it always stands 0.458 m
  from them. Wider ball-basket spacing does the same. All 9 layout failures in the map were the knee, never the grasp.
- Consequence for the real robot: the VLA learns arrival errors up to ~5 cm. Real navigation may be worse, so a short final positioning
  step with the head camera is likely needed there.

### 10.46 Keeping only successful demos can hide a blind spot (2026-10-10)
- The recorder keeps only successes. If the teacher fails in one region (here: ball and basket closer than 50° as seen by the robot;
  the right wrist touched the basket), that region silently disappears from the data. The VLA never sees it and fails there later.
- Fix: make the region a **rule of the sampler** (ball and basket ≥ 50° apart), so the starts we draw = the starts that end up in the
  dataset. 85 % as a gate also limits how much can hide in the failures. Result: 199/200.

### 10.47 The centre phase: correcting with feedback before the drop (2026-10-10)
- Turning the waist moves the ball on a circle around the waist axis. If the robot stands 4 cm sideways, ball and basket are at
  different distances from that axis, so the ball arrives up to 3.8 cm off the basket centre.
- New 1 s phase `centre`: both hands shift the ball horizontally over the centre before letting go (in sim from the true positions, on
  the robot it would come from the head camera). p95 3.8 → 0.3 cm. It also teaches the VLA a small "look and correct" motion.

### 10.48 The Samsung T7: when it is needed (2026-10-09)
- Not for Step 1 (it only writes small result files). **Plug it in right before the first 5 test demos** and keep it plugged in
  through recording, conversion and the training night: training reads the images from it.
- Size: 300 demos ≈ 30 GB raw + 30-40 GB converted (lossless images) ≈ 70 GB. The Mac has 133 GB free, the T7 ~890 GB.
- exFAT (the T7's file system) makes hidden `._` files on macOS; the recorder skips them. Plug in directly, not through a hub; keep the
  Mac on power and awake during long runs; never run the tests while recording.

### 10.49 Teacher and student: what is built and what is trained (2026-10-14)
- **Teacher** (`g1sim/ball_grasp.py`, programmed, never trained): it "cheats" - it reads the true ball position from the sim and computes
  the arm angles with IK. It only exists to make examples. It never runs on the real robot.
- **Demo recorder** (`69`): puts the robot at a random spot in front of the table (stop pose + a few cm/degrees of arrival error, random
  ball/basket, light, physics), lets the teacher work and records 30 times a second: head image (200x320, what the real D455 would see),
  27 joint values, the 27 commanded targets, and the sentence "put the volleyball in the basket". Only successes are kept.
- **Student = SmolVLA** (a ready-made 450M VLA, fine-tuned, not trained from zero): learns by imitation (behaviour cloning) image + joints +
  sentence → the next 50 actions. Afterwards it must do the task **without** the teacher's cheat - only camera and own joints, like the
  real G1. Walking is not trained (Unitree/GR00T controller); the VLA takes over at the "handover" at the table.

### 10.50 The four checks every demo passes (2026-10-14)
1. Teacher success (the same 7 checks as 67: no forbidden contact, lifted ≥ 6 cm, carried, 3 s in the basket, on its floor, at rest,
   hands off). 2. Format (`episode.validate`: image + 27 + 27 per frame, no NaN, hands 0..1, timestamps every 1/30 s, task text).
3. Plausibility (600-800 frames; measured waist follows its command within 0.05 rad). 4. Replay (a fresh sim loads the saved start
   snapshot and replays the actions: identical frame by frame). These test the *data*, not a model - nothing is trained yet.

### 10.51 The handover snapshot and "bit-exact" (2026-10-14)
- Each demo stores the full sim state at its first frame (positions, speeds, controller targets, the random-noise generator state).
  Restoring it and replaying the recorded actions gives the same images and joint values to the last bit ("bit-exact").
- Why it matters: the later eval can start the VLA from *exactly* a training situation (is it undertrained?) and compare it with the
  teacher from the same start. Small trap found: rendering a second camera view changed the *next* head image by up to 7 grey levels
  (state left in the graphics context) - one throw-away render fixed it.

### 10.52 Seven arm joints, one spare: why the IK "chooses" a posture (2026-10-14)
- Placing a hand (position + orientation = 6 numbers) needs 6 joints; the G1 arm has 7. The spare freedom (roughly: where the elbow
  goes) is chosen by a weak "posture task" that pulls towards a reference pose.
- The lift used to push both hands 9 cm straight up with a fixed orientation → the IK bent the elbows (a "bicep curl", forearms 39° up).
  Now both hands turn together about the line through the shoulders (like a forklift), with a small correction so the ball still goes
  straight up, and the reference pose during the lift is the arm at the squeeze → forearms straight forward, less curl. Making the
  elbow "stiff" in the IK was worse (elbows flared out) - measure each idea, keep only what the numbers and the video both support.
- Measure, don't guess: the first idea ("the 10-07 motion was different") was wrong - the probe showed the same posture in the same
  scene; the difference was where the robot stands (walk world: 4.3 cm lower, 3.8 cm further back).

### 10.53 What lives in runs/ (2026-10-14)
- `runs/` is the evidence folder, not in git. `.log` = full printed output of one run; `.json` = machine-readable result (every trial,
  its checks and variation); `.png` = images to check by eye; `labview/` feeds the browser Lab View (live image + status, finished
  records, replay videos); `demos.nosync/` and `teleop_check/` = old test recordings (~760 MB, deletable).
- Names: `l1w_s5_…` = Level 1 walk-up, Step/Stage 5. The `.lock_s<seed>` files are on the T7 while a recorder runs: they stop a second
  process from writing the same seed into the same folder.

### 10.54 Why a live view can slow a run down, and why parallel GPU starts can hang (2026-10-14)
- The live image at 10 per second made 4 parallel recorders 3.5x slower (93 s instead of 26 s per demo); one image every 2 s costs
  almost nothing. Measure with and without before blaming the physics.
- Starting 4 processes in the same second, one hung on its first render inside macOS's shared shader cache (a file lock). Start
  parallel GPU processes a few seconds apart.

### 10.55 How much fits on the T7 (2026-10-14)
- 890 GB free, one demo ≈ 117 MB raw → ~7,500 raw demos; with the converted LeRobot copy (estimate 100-130 MB each, not yet measured)
  ~3,500. 300 demos ≈ 35 GB raw + 30-40 GB converted, under 10 % of the T7. Time, not space, is the limit (~33 min per 300 in 4 processes).

### 10.56 Reviews can be wrong in their side effects too (2026-10-14)
- An independent reviewer tested a guard by running the recorder - the T7 had just been plugged in, so it recorded into the same folder
  with the same seed as my run and emptied my file. Lesson: reviewers read and check, they do not write into the data folder; and any
  shared output folder needs a lock.

### 10.57 Raw demos vs. LeRobot dataset, and why the dataset is not on the T7 (2026-10-14)
- The recorder writes **raw demos** (`.npz`, one file per demo, our own format). SmolVLA reads only the **LeRobot dataset format v3.0**
  (Parquet tables with the images inside, plus `meta/` with info, tasks and statistics). `79` converts, `80` checks that the LeRobot dataset
  is *exactly* the raw data (every joint and action value, statistics, images).
- The T7 is formatted exFAT (readable by Mac and Windows). exFAT cannot store macOS extra file information, so macOS writes a hidden
  `._name` twin next to every file. LeRobot reads "all `*.parquet` files" and crashes on `._file-000.parquet`. So: raw demos on the T7 (they
  are read one by one, by exact name), the LeRobot dataset on the Mac's own disk, and a one-file archive of it on the T7 as backup.
- A check is only worth something if it can fail: swapping two demos made the validator report 60 problems.

### 10.58 Why training takes 11 hours, and what a "step" is (2026-10-15)
- One training **step** = the model looks at 8 random frames (image + joints + sentence), predicts the next 50 actions for each, compares
  them with what the teacher did (the **loss**) and nudges its weights a little. On this Mac one step takes ~0.9 s.
- 45,000 steps × 8 frames = 360,000 frames ≈ 1.7 passes through our 217,800 frames. LeRobot's own SmolVLA recipe uses ~1.3 million
  (20k steps × 64) and takes ~4 h on one rented A100 GPU - on this Mac that would be ~40 h. So 45k is already the small version.
- We do not have to wait for the end: a **checkpoint** (saved model) is written every 5,000 steps and can be tested in the sim.
- Faster = a rented GPU (~10x) or a team member's NVIDIA machine; the dataset and script are ready for that.
- A falling loss means "imitates the teacher better on frames it has seen" - it does not prove the robot can do the task alone; only the
  evaluation in the sim (Step 8) shows that.

### 10.59 Why closing the window stopped the training (2026-10-15)
- A program started from a terminal/Claude Code session is a "child" of that session; when the session ends, its children are stopped.
  `nohup ... & disown` detaches it (its parent becomes the system), so it keeps running. `caffeinate -i` stops the Mac from sleeping.

### 10.60 How the trained VLA is tested in the sim (Step 4, 2026-10-15)
- Two programs talk over a local network connection: the **policy server** (`81`, runs the VLA) and the **sim client** (`82`, runs the
  robot). Every 25 ticks the client sends "camera image + 27 joint values + the sentence", the server answers with the next 50 moves, the
  client executes 25 of them, then asks again. Two programs because the VLA and the simulator need different Python environments.
- Before trusting a score, the pipeline is proven with **replay**: the server sends back a recorded demo's own moves. If the robot then does
  exactly what it did in the recording (same tick for lift, carry, placing), the pipeline is right. The server also checks that what it
  receives equals the recorded image/joints/sentence - otherwise a wrong camera image would look like "the model is bad".
- **Mutation test** = break the code on purpose (e.g. send an old image) and check that the gate fails. A check that never fails proves nothing.
- Success needs: lifted ≥ 6 cm, then all 4 placement checks for 3 s in a row, without touching table, stand or basket.

### 10.61 First results: the VLA alone, and how to read "skill" (2026-10-15)
- 20k checkpoint, 2 new situations in the sim: 1 success (ball in the basket after 21 s, no teacher), 1 failure (little finger brushed the
  ball stand while reaching). 2 trials are no statistic (could be 9-91 %), but the whole chain works with a real model.
- Open-loop check: the model predicts the next moves for 20 recorded moments; "skill" = how much better than just holding still (0 = no
  better, 1 = perfect). Arms 0.41 → 0.68 and waist 0.44 → 0.66 from 5k to 20k: still rising, so more training is worth it. These are
  training moments - only the sim test on new situations shows whether the model generalizes.


### 10.62 Why the closed-loop eval is still needed after the open-loop check (2026-10-16)
- Open-loop = the model only predicts from *recorded* moments. Closed-loop = the model drives the sim, so each small mistake changes the next
  image and the next mistake builds on it (compounding error). Only closed-loop shows whether the ball ends up in the basket.
- The 50 new trials answer four things: (1) the Level 1 number (success rate with a 95 % range, ~±13 points at 50 trials); (2) which
  checkpoint goes to the robot (40k vs 45k, the open-loop plateau cannot tell them apart); (3) undertrained or generalization gap (10 training
  starts vs 50 new starts → train longer vs record more varied demos); (4) whether the VLA is worth trying on the real robot at all.
- The teacher on the same 50 starts is the upper bound: failures the teacher also has are the scene's fault, not the model's.

### 10.63 First real number: 70 % on new situations, and what it means (2026-10-16)
- The VLA alone put the ball in the basket in 35-36 of 50 new situations (70-72 %, the 95 % range is about 56-83 %). The teacher did 50/50 on
  the same situations, so the scene is solvable and every failure is the model's.
- On 10 situations from the training demos it got 9-10/10. Good at what it saw, weaker at what is new = **generalization gap**. The fix is more
  varied demos (or narrower start ranges), not longer training. 40k vs 45k: 72 % vs 70 % is noise at 50 trials; the plateau seen open-loop is real.
- Most failures: a little finger brushes the ball stand while reaching. The rule is strict (any touch ends the trial), the same rule the teacher passes.

### 10.64 Real demos vs. more sim variety; only the grasp is learned (2026-10-16)
- **Real demos** = recordings on the physical robot: a person guides the arms (VR, by hand in a soft mode, or a replayed sim motion) while the real
  camera and joints are recorded in the same format as the sim demos. Not a test - data. Even 30-50 mixed into the sim demos teach the model
  what reality looks like. **More sim variety** (light, colours, textures) is cheaper and makes the model robust, but stays simulation.
- Only the grasp at the table is learned (VLA). Walking (Unitree controller), going to the table and back (navigation), aligning, task parsing
  and the state machine are ordinary code. So no separate dataset is needed for walking or for different rooms - the VLA only looks at the table.
- Order agreed in discussion: harden Level 1 first (more + more varied sim data, real demos, tests and evals, a numeric finish line), then Level 2.

### 10.65 Tests vs. evals, and the test layers we have (2026-10-16)
- **Test** = proves the code does what it should (yes/no, every run the same). **Eval** = measures how good the learned model is (a rate, e.g. 72 %).
- Layers in `tests/` (235 test functions, 33 files, `scripts/run_tests.sh`): **unit** = one function, pure logic, seconds (e.g. "the 27-D
  layout matches Unitree's order") · **integration** = several parts together, e.g. building the sim model and stepping it · **e2e** = the whole
  chain (walking controller + DDS + hands), minutes.
- A test is only worth something if it can fail: break the code on purpose and check the test goes red (mutation check).
- Missing so far: coverage numbers (which lines are run by a test), automatic runs on every push (CI), tests for the converter/validator/server
  (79-81).

### 10.66 Sharing a trained model with the team (2026-10-17)
- The robot needs code + the trained model (865 MB) + a fast laptop (Apple chip or NVIDIA), not the 60 GB of demos - those are only for training.
- Code goes through `git push`; the model goes into a **GitHub Release** (a file attached to the private repo, up to 2 GiB per file). Only repo
  collaborators can download it. Upload big files in two steps (create the release, then `gh release upload --clobber`), so a failed upload can
  simply be repeated. Teammates then download ~4-7 GB in total (model + SmolVLA base + Python packages): 30-60 min with normal internet.

### 10.67 "Pipeline works" is not "Level 1 done" (2026-10-27)
- Level 1 has a numeric finish line: held-out sim ≥ 85 %, walk mode ≥ 80 %, real robot ≥ 3/10. Today we have 72 % (36/50), no walk-mode number
  and 0 real attempts. The 95 % confidence interval of 72 % is 58-83 %, so even with luck the 85 % is not reached: the gap is real, not noise.
- The fix is more varied demos with a cleaner teacher (little-finger clearance), not more training steps: training starts are ~90-100 %, new
  starts ~70 % = generalization gap (10.63).
- Estimating new work from old work: count *active* days (7 for the Level 1 sim part), then multiply by how much longer things really took than
  planned (2.3-4.5x here). Reused code makes Level 2 cheaper (factor ~0.6-0.7); Level 3 is mostly new code (factor ~0.3 reuse).

### 10.68 Why a teacher needs margin, and why v1 and v2 must not be mixed (2026-10-27)
- The student (VLA) copies the teacher's path but never exactly. If the teacher's little finger passes the stand at 0.05 cm, any small
  deviation of the student becomes a contact. So we moved the teacher's path to ≥ 3 cm: same task, safer copy.
- v2 demos are recorded only with the new teacher. Mixing old and new demos would teach two grasps, and the average of both is the old
  near-miss path again.
- The table check (87) found the robot leans forward ~0.13 rad. Ignoring a lean makes a safety check optimistic: always check the worst case
  that was really measured.


### 10.69 Training steps vs. robot motion, and why more steps did not help (2026-11-01)
- A training step = the model sees a batch of 8 frames, guesses the teacher's next motion, and nudges its weights to shrink the error
  (the loss). 45k steps x 8 = 360k frames = ~1.65 passes (epochs) over v1's 217,800 frames. It has nothing to do with how far the robot moves.
- run1 plateaued from ~35k steps and was ~90-100 % on training starts but ~72 % on new ones: a generalization gap. The cure is more and
  more varied data, not more steps.

### 10.70 Domain randomization: what we vary and why (2026-11-01)
- Each demo draws new values inside ranges (configs/randomization.json): table height, textures, light, head-camera pose, control latency,
  object and stance positions. The model learns not to rely on any exact value, so the real robot becomes "just another variant".
- Too narrow: the real robot falls outside. Too wide: harder to learn and the teacher may fail. That is why every range change re-runs the
  teacher gates: latency 0-100 ms and table +4 cm failed them, so we chose 0-67 ms and -4..+1 cm.
- Latency = time from command to motor. If the sim reacts instantly but the real arm lags, the policy overshoots.

### 10.71 Sim-only artefacts are bugs in the data (2026-11-01)
- Anything the sim camera sees that the real camera never sees (debug marker dots, the camera's own housing as a bar) is a feature the model
  may latch onto and then miss on the real robot. Look at the recorded images, not only at the numbers: the bar was found in the lab view.
- Fix = hide them from cameras (render groups), then prove the physics did not change (same trials, same numbers).

### 10.72 How many demos, and what a sim success rate means (2026-11-01)
- 800 demos is an estimate (v1 300 -> 72 %, plus four new variations), not a promise; the eval decides. If training starts are high but new
  starts are low, record more (same teacher and setup may be mixed; old-teacher v1 may not).
- A high sim success rate is necessary, not sufficient: it counts only under wide variation, on unseen starts, plus Sim2Sim through the
  real adapter and shadow mode on real frames.

### 10.73 Test layers for a learning pipeline (2026-11-01)
- Unit tests: pure logic (latency queue, config, slices). Integration: stages fit together (record -> convert -> validate bit-equal, replay,
  wire contract, sim vs. real format). End-to-end: the whole pipeline in ~15 min before every big run. Success rates are acceptance gates,
  not pytest tests. Each new test is shown failing once on purpose (mutation) so we know it can catch a bug.

### 10.74 Waypoints + VLA: where the intelligence sits, and why the check takes long (2026-11-06)
- Waypoints = fixed, pre-checked stops for the arms (factory pose -> swing back -> hands up beside the body -> ready pose). They are the
  safe frame around the VLA, like a crane that always lifts before it swings. The intelligence is the VLA grasp/place in between: it reacts
  to where the ball really is. Visit 1: waypoints + VLA grasp. Visit 2: walk-up + grasp/place.
- 87 computes every hand and finger point along each route (2 cm apart) for many elbow poses, torso leans and table heights, so it takes
  minutes. A route counts as safe only if it stays >= 5 cm from table, stand and basket in every case, including the upright torso.
- "86 recover" brings the arms home after a crash, but only if the arms are still where the last log says. Otherwise it refuses and sends
  nothing: a robot that refuses is safe; a robot that guesses where its arms are can drive them into the table (the review found -9.9 cm).
- Each fix was reviewed by an agent that did not write it and re-ran the checks itself; that is how the two recover bugs were found.

### 10.75 Demos are the textbook, training is studying, eval is the exam (2026-11-08)
- Recording demos only writes the textbook. The model learns during training, when it goes through the examples again and again. What
  counts is how often it has seen each example (epochs), not the number of steps: steps x batch / frames = epochs.
- run2: 90k steps x 8 = 720k samples on 1,227,666 frames = 0.59 epochs: it never saw half the book once. The steps were planned for 800
  demos and not re-computed when the set grew to 1,691. A low loss (0.012) did not show it; the exam did: 47 % on new starts, and only
  3/10 on starts it trained on ("undertrained": more training helps, more demos would not).
- The eval report per trial: success (ball in basket, held 3 s), how far it got, why it failed (forbidden contact = safety, not lifted,
  not placed in time), times. Totals with a 95 % range (100 trials = about +-9 %), training starts vs new starts, teacher as upper bound.
- Safety is a separate layer, not part of the model: the VLA only imitates and does not "know" the table, so every command passes limits,
  the demo corridor, speed caps, contact detection and (new) a virtual fence before it reaches a motor. Training and safety can be built in
  parallel; they meet in the Sim2Sim rehearsal.
