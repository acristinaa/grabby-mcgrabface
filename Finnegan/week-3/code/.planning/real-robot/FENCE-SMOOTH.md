# FENCE-SMOOTH: virtual fence + chunk-boundary smoothing for the 86 policy phase

Planner, 2026-11-08, **revision 2** (answers plan review round 1 in `.planning/reviews/fence-smooth.md`). Status: **built
2026-11-08 (implementer, section 6), not committed, code review open; E2 (85 replay with the fence) fails - see 6.3.**
Visit 1 at SAP: Fri 2026-11-13.
Goal (Finnegan): the real G1 runs the learned grasp, never hurts itself or hits table/stand/basket, no violent jerks
(a bit jerky is fine), tries to pick up the ball; a miss is acceptable.

All numbers were measured on the Mac in sim/numpy (no VLA model loaded, one sim process at a time). Raw files:
`/tmp/fence/teacher50/` (50 teacher trials, v2 config, seed 0), `/tmp/fence/b2/` (replay demo layout, 50 training demos),
`/tmp/fence/b1/` (fence model file), `/tmp/fence/m5/` (smoothness per trial), `/tmp/fence/smooth2/`, `/tmp/fence/review/`
(reviewer: freeze probe, blend replay). Scripts: `/tmp/fence/scripts/`.

## What changed in revision 2 (review round 1 -> fix)

| Finding | Fix in this plan | Where |
|---|---|---|
| B1 robot laptop cannot build the fence model | ship a collision-only model `g1sim/data/fence_model.xml.gz` (0.54 MB, measured equal to the full model within 0.001 mm); pin `mujoco==3.13.0` in the robot env; model file in the motion gate; 89 step; 86 refuses off loopback without it | 1.1, 1.6, 3 (B0, B4, B8) |
| B2 margin assumes exact layout; replay gate fails | **new measurement**: the replay demo's own layout found (dataset ep 0 = s101 t0, image-checked); calibration now on the **training demos of the shipped model** (they come much closer than the teacher); **per-group margins**; error budget; tests run in the demo's own layout; robot-day replay puts the stand where the demo had it (R-F8) | 1.2, 1.3, 4 (I6, E2) |
| B3 blend breaks the replay gate | no blend in `replay` (only `run`); U8 proves it | 2.3, 4 (U8) |
| M1 upper arm not fenced | moving parts for table/stand/basket = everything below `*_shoulder_pitch_link` (87's set) | 1.1 |
| M2 hold does not stop the arm; no escape | new rule "**never closer than holding still**": a command below the margin is still sent if it is not closer than the held pose; streak counts only refused ticks; overshoot numbers in 1.5 | 1.5 |
| M3 hands in pred | both poses use the **commanded** hand target; only arms/waist get the tracking error; missing hand state never holds | 1.1, 1.5 |
| M4 shadow fence meaningless | shadow checks `cmd` only, labelled "informational"; R-F5 (shadow abort) dropped; the operator check is the replay's fence summary with live `pred` | 1.5, 1.7 |
| m1 no hand args in `SafetyFilter.step` | new signature in B2/B3 | 3 |
| m2 no yaw in the layout | robot yaw on the marks is an entry (and in the budget); stand/basket have no yaw in sim (not randomized), so the runbook places them square | 1.3 |
| m3 distractors not calibrated | E1 (teacher `--fence`) mandatory before any VLA comparison | 4 |
| m4 85 has no table | contact claim moved to I7 (82 sim) | 4 |
| m5 pooled smoothness bar | bar is now **per trial, moving ticks only**: teacher p95 17.1 rad/s^2 | 2.1 |
| m6 holds and smoothness | 82 `--fence --blend` reports p99 \|a\| including holds | 4 (S2) |
| m7 envelope finding | still flagged, not in scope | 5 |

---

## 0. What is protected today, what is not

| Phase | Protection | Where |
|---|---|---|
| ramp-in, way in, way out, soft exit, recover | fixed waypoints, every 50 Hz tick checked offline by 87 against table/stand/basket with >= 5 cm | `g1sim/arm_paths.py:83-107`, `g1sim/arm_clearance.py:16`, `g1sim/arm_clearance_fk.py:49-155` |
| policy (replay, run) | joint limits, demo envelope, waist clip, rate limit 1 rad/s, lead monitor 0.25 rad / 13 ticks | `g1sim/real_safety.py:275-327` |
| policy: distance of arms/hands to table, stand, basket, own body | **nothing** | - |

Evidence for the gap: run2 90k sim eval, 100 held-out starts, 16 failures with forbidden contacts (PROGRESS.md:47-49);
run1 45k: 12 (left pinky on the stand 6) (PROGRESS.md:1065). The lead monitor only fires *after* a contact has blocked the
arm for 0.26 s.

**What the fence can and cannot promise on the real robot (plain words).** It knows the robot's own body exactly, so the
self part (torso, head, legs, other arm) is a real guard. For table, stand and basket it only knows where the operators
say they are; with the estimated layout error (1.3, about 1-2 cm) and the arm's overshoot after a stop (up to 1.4 cm in
sim) it **cannot promise "no touch"**. What it does: it stops the policy from pushing a hand on toward an object - a push
ends within a few cm of the fence line instead of "until the arm is blocked for 0.26 s". The lead monitor stays the
backstop.

---

## 1. Virtual fence

### 1.1 What it computes (one pure function, two constructors)

New module `g1sim/fence.py` (pure FK + distances, no DDS, no I/O):

```
class Fence:
    @classmethod from_env(env)          # sim (82): the eval env's own model, a private MjData, base pose copied per tick
    @classmethod nominal(layout)        # real (86): g1sim/data/fence_model.xml.gz + layout (same keys as a sim variation)
    def values(q17_cand, hands12_cmd, measured17, last17, waist_rp, pelvis_rp) -> {group: metres}   # min over cmd and pred
    def held_values(last17, hands12_cmd, measured17, waist_rp, pelvis_rp) -> {group: metres}      # "hold still" pose
    MARGIN = {"table": 0.013, "stand": 0.020, "basket": 0.009, "self": 0.020}
```

- **Moving parts**: for table, stand, basket (and FORBIDDEN distractors) = every collision geom below
  `left/right_shoulder_pitch_link`, the set 87 uses (`arm_clearance_fk.py:19`; M1: the teacher's upper arm comes to 5.08 cm
  of the basket, the training demos to 4.97 cm). For the self groups = every collision geom below `*_elbow_link` (the
  upper arm touches the torso by construction).
- **Scene groups** = the bodies of `ball_grasp.FORBIDDEN` present in the model (`g1sim/ball_grasp.py:102`: table,
  volleyball_stand, basket, distractors in sim) - the sim's forbidden-contact list (`g1sim/vla_eval.py:164-169`). **The
  volleyball is not in it.**
- **Self groups** = torso (`torso_link`, `backpack_link`), head (`head_servo_link`, `d455_link`), pelvis + hips + knees, and
  the other arm (subtree of the other `shoulder_yaw_link`).
- Distance = `mj_geomDistance` after a bounding-sphere pre-filter, capped at 0.10 m (`arm_clearance_fk.py:144-155`).
- **Pose that is checked = min over two poses** (M3 fixed):
  - `cmd`: the 17-vector about to be sent; hands at their **commanded** target;
  - `pred` = `cmd + (measured - last_sent)` for **arms and waist yaw only**; hands at their **commanded** target (BrainCo
    fingers lag up to ~1 s and then arrive; "sagged arm + closed fingers" is the case to check). Waist roll/pitch = measured
    (motors 13/14; env qpos in sim). Pelvis roll/pitch = **LowState IMU** on the robot (new, one field read in `real_io`;
    the sim uses its own base pose); pelvis position and yaw = the layout entry.
  - Why both: the PD arm sags. Teacher 50 trials: `|measured - commanded|` arms p99 0.183 rad, max 0.278 rad. The commanded
    pose never comes closer than 10 cm to the table, the real arm comes to 4.5 cm. `cmd` stays in the min because it sees a
    command pushing through something the arm is blocked by.
- Non-finite measured arm/waist angle or IMU roll/pitch -> the tick is refused (fail closed; `lowstate_stale` normally faults
  first). Missing or NaN **hand state never matters** (the fence uses the commanded hand target), so `--no-hands` and
  `hand_state_synthetic` runs are not affected (M3).

**The fence model on the robot laptop (B1).** 86 today loads no MuJoCo, and `TableEnv` reads its XML from
`~/dev/g1-ext/g1_brainco_sim` (outside the repo, not on the robot laptop). So:
- `tests/helpers/make_fence_model.py` (sim env) builds a collision-only copy of the `volleyball_walk` scene: visual geoms and
  textures removed, every collision mesh replaced by its convex hull vertices written inline (MuJoCo collides meshes by
  their hull anyway). Written as `g1sim/data/fence_model.xml.gz` with the source scene's sha256 and the MuJoCo version in a
  header comment. **Measured** (`/tmp/fence/b1/hull.log`): 103 geoms, 67,912 hull vertices, XML 1.95 MB, gzip **0.54 MB**,
  load 0.06 s; distances to table/stand/basket on 145 teacher poses equal to the full TableEnv model within **0.0006 mm**.
- `Fence.nominal(layout)` loads it with `mujoco.MjModel.from_xml_string` and applies the layout the way `TableEnv._set_up` /
  `_pose_base` do (stance x/y/yaw, ball+stand offset, basket offset, table dz). I4 pins "nominal == TableEnv" on 20 random
  v1 and v2 variations.
- Robot env: `ROBOT_PINS` adds `mujoco==3.13.0` (the Mac's version; `mj_geomDistance` results depend on it). PyPI has
  3.13.0 cp310 wheels for **macOS arm64 and Linux x86_64 only - no Intel-Mac wheel**. Dry run into the existing py3.10 test
  env with `numpy==2.2.6`: resolves (adds absl-py, etils, glfw, pyopengl). `fence.py` refuses any other MuJoCo version.
- Motion gate: `loaded_repo_files` lists modules only, so 86 adds `fence.DATA_FILES` (the model file) to the list it checks;
  an edited model file blocks motion like an edited module.
- 86 refuses `run` and `replay` off loopback when the fence cannot be built (no mujoco, wrong version, file missing, layout
  missing). `--no-fence` exists for loopback tests only and is refused off loopback.

### 1.2 Calibration: the training demos of the shipped model, not only the teacher (new measurement)

Revision 1 calibrated on 50 current-teacher trials (v2 config). But the model in the release (`smolvla_run1_040000`,
`setup_robot_laptop.sh:22`) learned from the **v1 demos** (`g1_volleyball_walkup_v1`, 300 demos). I regenerated the layouts
of 50 of them (every 6th episode; v1 config, the demo's seed, reset index = trial) and checked each by its head frame 0:
mean pixel difference median 3.55, max 4.50 (a wrong layout gives 43-46). Fence values with the rev-2 `pred`, scene groups
including the shoulder links (`/tmp/fence/b2/demos50.log`):

| group | v2 teacher, 50 trials: min | v1 training demos, 50: min / p5 / median | demos with any action < 2 cm | closest part |
|---|---|---|---|---|
| table | 4.46 cm | **1.52** / 1.82 / 2.51 cm | 8 / 50 (up to 23 actions) | forearm/hand, `pred` (cmd >= 10 cm) |
| stand | 3.70 cm | 2.96 / 2.98 / 3.08 cm | 0 / 50 | forearm/hand |
| basket | 3.60 cm (upper arm 5.08) | **0.59** / 1.39 / 3.08 cm | 9 / 50 (up to 37 actions) | forearm/hand, `pred` (cmd >= 3.99 cm; upper arm >= 4.97) |
| torso | 4.04 cm | 4.04 / 4.04 / 4.07 cm | 0 | |
| head, pelvis/legs | >= 10 cm | >= 10 cm | 0 | |
| other arm | 5.76 cm | 5.32 / 5.48 / 5.91 cm | 0 | |

The demos come much closer than the current teacher (cause not checked; these demos have arrival yaw up to 12.5 deg).
Revision 1's 2 cm on every object would have refused approach in 17 of 50 demo-like motions.

**Margin rule (R-F1):** per scene group, margin = training demos' p5 minus 0.5 cm, capped at 2.0 cm, at least 0.5 cm;
self groups 2.0 cm. For the v1 demos: **table 1.3 cm, stand 2.0 cm, basket 0.9 cm, self 2.0 cm.** With the "never closer"
rule (1.5) a demo-like motion below a margin is slowed (approach refused), not stopped dead. Only 1 of 50 demos goes below
its group's margin in its own layout (basket 0.59 cm). The stand gets the full 2 cm: it is where the sim failures happen
(run1: left pinky on the stand 6 of 12) and the demos keep >= 2.96 cm. **If another checkpoint ships (run3 on v2 data), the
rule is re-run on that dataset (I8)** before the motion unlock.

### 1.3 Layout entry and error budget (B2, m2)

Layout = the same keys as a sim variation: stance `x_m, y_m, yaw_deg` (robot vs table), ball/stand `dx_m, dy_m`, basket
`dx_m, dy_m` or none, table `dz_m`. 86 takes it as sheet numbers in the robot's frame (`--fence-setback-cm`,
`--fence-table-dz-cm`, `--fence-stand-dx-cm/-dy-cm`, `--fence-basket-dx-cm/-dy-cm` or `--fence-no-basket`,
`--fence-robot-yaw-deg`, default 0 = feet on the marks) or as `--fence-layout FILE.json` (tests, replay demo). One function
`layout_from_sheet()` converts; I3 tests it.

Distance between two shapes changes by at most the length of the move (1-Lipschitz), so every cm of layout error is at
most one cm of fence error; the reviewer measured about 1:1 for the stand (`demo_nominal_walk2.txt`). Estimated errors on the
robot day (**estimates, not measured on the robot**):

| source | how it is known | est. error | groups |
|---|---|---|---|
| table top height | B3 formula from the hip height, re-checked in Part C (1 cm) | 0.5-1 cm | table (+ stand/basket height) |
| table front edge | string + tape, Part C check "within 1 cm" | up to 1 cm | all scene |
| stand/basket on the table | tape from edge/centre line; stand also in the 84 camera overlay (1 px is about 2.4 mm at 0.5 m) | 0.5-1 cm | stand, basket |
| robot yaw on the marks | toe/heel tape; overlay shows the table edge | about 1 deg -> 0.8 cm at the stand (0.46 m from the pelvis) | stand, basket |
| pelvis roll/pitch | IMU (new) | about 0.5 deg -> 0.4 cm | all scene |
| basket yaw | placed square (not randomized in sim) | 3 deg -> 1 cm at a rim corner | basket |
| fence model vs sim | measured 0.0006 mm | 0 | - |
| overshoot after a hold | sim speed 1 (1.5) | stand up to 1.4 cm, table 0.5, basket 0.6 | all scene |

Root-sum-square of the layout rows: about 1.5 cm for the stand. So on the robot: a demo-like approach gets refused when the
error points toward the object by more than ~1 cm (stand) - a slowed grasp, not a crash; and a push toward an object ends
within about margin - error - overshoot of it (between ~2 cm before and ~1-2 cm into it in the worst case). That is the
honest claim.

**Operator check before live (B2, replaces the shadow fence):** the 86 replay runs with the fence on and the real `pred`.
At its end 86 prints per group the smallest fence value, holds, and overshoot (1.5). Expected for the replay demo with the
stand placed as in R-F8: table 2.31, stand 3.54, basket 3.02 cm, 0 holds (FK with the demo's recorded states,
`/tmp/fence/b2/replay_stand_moved.log`). A hold or a value far below that in the replay = layout entry or robot pose wrong:
stop, re-measure, do not go live.

**Replay demo layout (new).** `tests/data/replay_demo.npz` is dataset episode 0 = `episode_s101_t0000` (convert map). Its
layout regenerated (seed 101, `configs/randomization_v1.json`, first reset): head frame 0 differs by 3.76 vs 42.9 / 46.4 for
the next draw / nominal (`/tmp/fence/b2/demo_layout.log`). Stance x -9.9 cm, y -9.4 cm, yaw 6.3 deg; ball dx -1.4, dy -6.3 cm;
basket dx +1.3, dy -3.9 cm; table dz 0. In the robot's frame the demo's stand sits 6.7 cm further back and 1.7 cm further
right than the nominal stand. Fence values of the demo:

| layout | table | stand | basket | torso | other arm |
|---|---|---|---|---|---|
| its own (s101 t0) | 1.97 | 3.27 | 4.18 | 4.09 | 6.55 cm |
| nominal (robot-day replay as the runbook has it today) | 2.31 | **1.70** (37 actions < 2 cm) | 3.02 | 4.09 | 6.55 cm |
| nominal + stand 6.7 cm back, 1.7 cm right (R-F8) | 2.31 | 3.54 | 3.02 | 4.09 | 6.55 cm |

So the real replay today already passes about 1.7 cm (model) from the stand. Stored as `tests/data/replay_demo_layout.json`
(variation + robot-frame offsets) by `tests/helpers/make_replay_demo_layout.py`, which refuses to write unless the head-frame
check is < 10.

### 1.4 Segment between ticks: not checked (with the reason)

- Per 50 Hz tick the rate limit allows 0.02 rad per joint (`real_safety.py:24, 287`). Measured largest geom movement per
  teacher action (30 Hz, speed 1): p50 0.22 cm, p99 1.02 cm, max 11.6 cm (the teacher's own 0.1 rad jump at tick 150).
  Through 86 at speed 0.5 that is ~0.3 cm per tick (p99); the rate limit caps the worst at ~2.3 cm per tick.
- A point moving on a straight segment of length s past a convex obstacle stays >= sqrt(m^2 - (s/2)^2) away when both ends
  are >= m away. With the smallest margin 0.9 cm the bound needs s < 1.8 cm; the p99 0.3 cm is far below, the worst-case
  2.3 cm is not. So **U9 pins the worst per-tick geom step under the rate cap against 2 x the smallest margin**; if it
  fails, the build adds a midpoint check (one more FK call on those ticks), not a bigger design.

### 1.5 What 86 does when the policy comes too close (M2 fixed)

Inside `SafetyFilter.step`, **after** the rate limit (`real_safety.py:287`) and **before** the lead monitor (`:290`), only in
phase `"policy"`:

1. `cand` = the final command (envelope, limits, rate limit already applied). `F(cand)` = per group min over cmd and pred.
2. If every group is >= its margin: send `cand`.
3. Else, for each group below its margin compare with `F(hold)` (the fence value of holding still: `cmd = last`,
   `pred = measured` arms/waist, hands commanded). **Send `cand` if it is not closer than holding still in every such group**
   ("never closer than now", the rule 87's `exit_passes` uses). Otherwise **hold**: `cmd = self.last`, event `fence_hold`,
   hands not updated, `self.last` unchanged.
4. `fence_streak` counts consecutive **held** ticks; a sent tick resets it. `fence_streak >= FENCE_STREAK_TICKS` -> event
   `fence_streak` -> `Runner._react` raises `ExitRequest("soft", "fence_streak")` -> the normal soft exit (way out), exactly
   like `envelope_streak`.

Why this fixes M2: after an overshoot the held pose itself is below the margin; the old rule then refused every step and
the arm sat until the streak exit. Now any step that moves away or along is sent, so the arm can leave, and the streak only
counts ticks where the policy keeps pushing in. That makes the streak length meaningful: **N = one chunk + 0.5 s**,
`ceil((EXEC_STEPS / (ACTION_HZ * speed) + 0.5) * SEND_HZ)` = 109 ticks at `--speed 0.5`, 192 at 0.25 (R-F2).

**Overshoot after a hold** (reviewer's freeze probe, sim, teacher at speed 1, 1,219 holds of 1 s, `/tmp/fence/review/
freeze20_stats.txt`): measured distance below the accepted fence value: stand p99 1.08, **max 1.40 cm**; table max 0.49;
basket max 0.62; torso never. Speed 0.5 not measured (expected lower). The robot's is unknown, so 86 logs per hold
`fence_overshoot` = value at the hold minus the smallest held-pose value over the next 1 s (sidecar, and the replay
summary).

Other rules:
- **Envelope** clips first; both streaks count independently.
- **Lead monitor** runs on the sent command; it still catches contact that happens anyway (`contact_fault`).
- **Faults** unchanged; a fence streak is a soft exit.
- **Scripted phases**: no fence (87 checked them with 5 cm).
- **Shadow** (M4): the shadow filter runs the fence on **`cmd` only** (the arm sits at the ready pose, so `pred` would be
  about the ready pose) and counts `shadow_fence_cmd`; the summary line says "cmd only, blind to arm sag, information". **No
  shadow abort rule for the fence (R-F5 dropped).** The real check is the replay summary (1.3).
- Hands: `PolicyPhase._target` no longer commands hands; it returns `(target17, hands12)`, `Runner.tick` passes both to the
  filter and commands the hands only when there was no `fence_hold` (m1).

**Per-tick compute** (M4 Mac): one FK + groups = 0.37-0.42 ms mean, p99 0.74-0.85 ms (`freeze20.log`, `teacher50/stats.txt`).
2 calls per tick (cmd, pred), 3 when below a margin (+ held pose; `cmd = last` is cached): ~1.2 ms mean, ~2.6 ms p99 of the
20 ms tick. The robot laptop must show it (E4, 89).

### 1.6 `82 --fence` (sim eval, same function)

In `vla_eval.run_policy_trial` (`g1sim/vla_eval.py:274-316`), before `env.apply(action)` (line 305): `Fence.from_env(env)`
with the same rule (pred from `env.state()` and the last applied action; hands commanded; refused action -> apply the last
applied action again). `FENCE_STREAK_ACTIONS = EXEC_STEPS + 15` -> trial ends `fence_exit` (a failure, not a forbidden
contact). Row fields `fence_holds`, `fence_first_tick`, `fence_worst`, `fence_exit`. The margins are the same constants.

**Where the VLA comparison runs**: not on the Mac (no VLA model here). On the Hive PC after run3, or on a rented GPU:
`82 --no-walkup --seed 0 --trials 100` with and without `--fence`, same checkpoint (S3).

### 1.7 What the operator sees (R-F4)

One line on the first hold of a streak ("FENCE: right hand 1.4 cm from stand - holding"), one on the exit ("soft exit
(fence_streak)"), counts and overshoots in the sidecar, and the replay summary (1.3). No per-tick printing.

---

## 2. Smoothness

### 2.1 Definition (m5 fixed)

Per joint (arms + waist yaw), on the 17-vector 86 sends at 50 Hz (ChunkScheduler + SafetyFilter): v, a, j with dt = 20 ms.
Statistic: **per trial, p99 |a| over moving ticks only** (worst joint |v| > 0.05 rad/s), worst joint. Pooling over trials
hid the spread (one teacher trial reached 15.9 on all ticks), and the still phases pulled the pooled number down.

**Too jerky = a trial above the teacher's 95th-percentile trial at `--speed 0.5`: 17.1 rad/s^2** (teacher 100 trials: median
12.0, p95 17.1, max 22.8; `/tmp/fence/m5/per_trial.log`).

### 2.2 Measured (speed 0.5)

| source | p99 \|a\| moving ticks (per trial) | verdict |
|---|---|---|
| teacher, 100 trials | median 12.0, p95 17.1, max 22.8 | the bar |
| VLA run1 45k, 1 trial, latency 0 | 22.9 | too jerky |
| VLA, latency 3 | 30.2 | too jerky |
| VLA + blend 5, latency 0 / 3 | 12.9 / 13.0 | fine (teacher median) |

At 0.25 (pooled, revision 1): VLA 11.0, with blend 4.9. Cause: the chunk boundary (new chunk vs old at the same moment:
median 0.016 rad, max 0.051 rad). Not violent: the biggest jump is half the teacher's own 0.10 rad, and the rate cap bounds
every jump. One VLA trial only; S3 repeats with run3 chunks.

### 2.3 Limiter: crossfade at chunk boundaries, **`run` only** (B3 fixed)

In `ChunkScheduler` (`real_timing.py:56-75`): keep the old chunk on `add_chunk`; for the first `BLEND_ACTIONS = 5` actions
after a switch, `target = (1 - w) * old(position) + w * new(position)`, w 0 -> 1 linearly; no blend when the old chunk has
run out. **`ChunkScheduler(speed, blend=0)` by default; only `PolicyPhase` in mode `run` passes 5.** Replay chunks are
slices of one demo, the blend buys nothing there and moved 4-6 of 2,417 logged actions off the demo polyline (reviewer,
`blend_replay_rows.txt`), which the replay gate (`CONTENT_TOL = 1e-4`) rightly refuses. 82 gets `--blend 5`.
- Effect: 22.9 -> 12.9 (latency 0), 30.2 -> 13.0 (latency 3) at 0.5.
- No acceleration cap (teacher max 62 rad/s^2 = VLA max; a cap adds lag to every move).

---

## 3. Build plan (smallest correct diff)

| Step | Files | Content |
|---|---|---|
| B0 | new `tests/helpers/make_fence_model.py`, new `g1sim/data/fence_model.xml.gz`; new `tests/helpers/make_replay_demo_layout.py`, new `tests/data/replay_demo_layout.json` | the collision-hull model (1.1) and the replay demo's layout with the head-frame check (1.3); both run in the sim env, once |
| B1 | new `g1sim/fence.py` | `Fence` (from_env, nominal + layout, values, held_values, MARGIN per group, `DATA_FILES`, MuJoCo version check), `layout_from_sheet()`, streak helpers |
| B2 | `g1sim/real_safety.py` | optional `fence` in `SafetyFilter.__init__`; `step(target17, measured17, phase, weight, hands12=None, waist_rp=None, pelvis_rp=None)`; fence stage between rate limit and lead monitor with the never-closer rule; `fence_hold`/`fence_streak`; overshoot bookkeeping |
| B3 | `g1sim/real_run.py`, `g1sim/real_run_policy.py`, `g1sim/real_io.py` | build the Fence from the layout; `_react` -> soft exit on `fence_streak`; `_target` returns hands, `Runner.tick` commands them only without `fence_hold`; IMU roll/pitch read from LowState; shadow counts `shadow_fence_cmd`; replay end summary |
| B4 | `scripts/86_real_run.py`, `g1sim/real_preflight.py`, `g1sim/motion_gate.py` call site | layout args / `--fence-layout`; refuse `run`/`replay` off loopback without a fence (no silent default); `--no-fence` loopback only; gate checks `fence.DATA_FILES` |
| B5 | `g1sim/real_timing.py` | ChunkScheduler blend (default 0; `run` passes 5) |
| B6 | `g1sim/vla_eval.py`, `scripts/82_vla_eval.py` | `--fence`, `--blend`; row fields; summary |
| B7 | `docs/ROBOT_DAY_RUNBOOK.md` | layout numbers into the 86 commands; replay stand placement (R-F8); replay fence summary as the go/no-go before live; what the operator sees |
| B8 | `scripts/setup_robot_laptop.sh`, `scripts/89_robot_day_selftest.py` | `mujoco==3.13.0` in `ROBOT_PINS`; robot-env block fails with a clear line on an Intel Mac; 89 step `fence`: mujoco version, model loads, 1,000 FK calls p99 < 5 ms on this laptop |

The motion lock (`docs/ROBOT_DAY_MOTION.json`) covers every file 86 loads plus the model file, so Finnegan unlocks at the
commit that contains B0-B5 and B8.

Out of scope: retraining, depth/LiDAR, Qwen, changing waypoint routes, the envelope literals (5).

---

## 4. Tests

| # | Test | Kind | Proves | Catches |
|---|---|---|---|---|
| U1 | stub fence "too close and closer than holding" -> sent == previous, `self.last` unchanged, `fence_hold` | unit | a pushing command is never sent | sending it anyway |
| U2 | N-1 held ticks no exit, N -> `fence_streak` once; a sent tick resets | unit | streak counting | off-by-one, missing reset |
| U3 | stub fence records its input = post-rate-limit command | unit | fence sees what is sent | checking the raw target |
| U4 | fence not called in to_ready/to_start/ramp/release; shadow counts `shadow_fence_cmd` from `cmd` only | unit | policy-only scope; shadow honest | fence on 87 routes |
| U5 | lead monitor gets the held command | unit | lead on what was sent | lead on the refused command |
| U6 | Runner: `fence_hold` -> hand command unchanged; `fence_streak` -> soft exit via way_out | unit | hands hold with arms; exit path | fingers closing near the stand |
| U7 | arm/waist NaN or IMU NaN -> hold; hand state NaN / `--no-hands` -> no hold, hands commanded | unit | fail closed where it matters (M3) | silent pass; every `--no-hands` run ending in `fence_streak` |
| U8 | blend: 0.05 rad step spread over 5 actions; identical chunks -> identical; ran-out old chunk -> no blend; **mode `replay` -> scheduler has blend 0** | unit | the limiter; B3 | blend in replay, inverted, never ends |
| U9 | worst per-tick geom step under the rate cap (all arm joints 0.02 rad, waist yaw 0.006, 20 teacher poses) < 2 x smallest margin | unit (FK) | no segment check needed | a rate-limit raise that breaks 1.4 |
| U10 | never-closer rule: below margin but farther than holding -> sent (escape); below and closer -> held; above margin -> sent | unit | M2 | arm stuck after overshoot |
| U11 | gate: edited `fence_model.xml.gz` -> `gate_problems` names it | unit | B1 gate | a swapped model |
| U12 | `fence.py` with another MuJoCo version or a missing file -> clear error; 86 off loopback refuses `run`/`replay` | unit | fail closed | silent no-fence run |
| I1 | known poses: ready pose >= margin everywhere; hand into table / stand / torso / other hand, **upper arm into basket** -> < 0 for that group | integration | each group and the shoulder parts are wired | a dropped group |
| I2 | pred: commanded pose 10 cm above the table, measured sag offset puts it at 1 cm -> refused | integration | the sag correction | cmd-only fence, sign flip |
| I3 | `layout_from_sheet`: setback +10 cm raises table distance ~10 cm; table_dz, stand/basket offsets, robot yaw move them; no basket -> no group | integration | layout entry | ignored args |
| I4 | `Fence.nominal(v)` (hull model file) == `Fence.from_env(TableEnv reset to v)` within 0.01 mm on 20 random v1 + v2 variations x 20 teacher poses | integration (sim env) | the shipped model and the layout code match the sim | wrong hull, wrong frame, two scene builds drifting |
| I5 | ball never an obstacle; FORBIDDEN distractors are | integration | obstacle list | ball counted |
| I6 | `replay_demo.npz` (states + actions) with `replay_demo_layout.json` -> 0 refused ticks; mins table 1.97, stand 3.27, basket 4.18 cm (+-0.05) | integration | calibration on the recorded demo in its own layout | margin too large; layout ignored |
| I6b | same demo, nominal layout + R-F8 stand placement -> 0 refused; nominal without R-F8 -> refusals at the stand | integration | the robot-day replay case | runbook placement wrong |
| I7 | 82 `--fence`, fake client drives a hand into the table -> holds, `fence_exit`, **no forbidden contact** (82 has the table) | integration | sim wiring + contact claim (m4) | fence off in 82 |
| I8 | margin rule on the shipped checkpoint's dataset: 50 demos in their own layouts (head-frame check < 10), margin <= p5 - 0.5 cm per group | integration (sim env, FK only, ~1 min) | calibration matches the model that ships | run3 shipped with run1 margins |
| E1 | `82 --teacher --fence --no-walkup --seed 0 --trials 50` -> 0 holds; **mandatory before any VLA comparison** (m3) | run (Mac, one process, ~2-3 min) | teacher (with distractors) never refused | over-tight margin |
| E2 | 85 e2e `test_replay_tracks_demo_inside_envelope` with `--fence-layout tests/data/replay_demo_layout.json`, no blend -> code 0, 0 fence holds, polyline check unchanged | e2e | replay gate holds with the fence | fence breaking the replay |
| E3 | 85 e2e new: replay a demo edited to lower a hand into the table -> holds, soft exit `fence_streak` (no contact claim: 85 has no table) | e2e | the real adapter path | missing `_react` |
| E4 | 85 run log + 89 `fence` step: fence time per tick p99 < 5 ms, message gap < 60 ms | e2e | budget | slow FK |
| S1 | stored run1 chunks through the blend: per-trial moving-tick p99 \|a\| at 0.5 < 17.1 | unit | smoothing claim | blend regression |
| S2 | 82 `--fence --blend 5` row: p99 \|a\| including holds (m6) | run (PC/GPU) | holds do not make it jerky | |
| S3 | PC/GPU: run3 82 with and without `--fence`/`--blend`, save chunks, recompute 2.2 | run (not Mac) | real effect on success and contacts | fence that kills success |

If E2 shows holds although I6 shows none, the 85 sim's arm tracks differently; report it with the numbers, do not tune the
margin silently.

Mutants that must go red: (1) fence stage removed (U1); (2) hold sends the new command (U1); (3) pred = cmd only (I2);
(4) pred sign flipped (I2); (5) `<` vs `<=` on the margin (I1); (6) streak not reset (U2); (7) streak N-1 (U2); (8) setback
ignored (I3); (9) other-arm group dropped (I1); (10) torso dropped (I1); (11) ball as obstacle (I5, I6); (12) fence in
to_ready (U4); (13) hands sent on hold (U6); (14) blend inverted (U8); (15) fence before rate limit (U3); (16) streak exit as
fault (U6); (17) blend on in replay (U8); (18) never-closer rule removed (U10); (19) shoulder links dropped (I1); (20) hand
state NaN holds (U7); (21) model file not in the gate (U11); (22) measured hands in pred (I2 variant with lagging fingers).

**Mac use for the build's checks** (one process at a time, no VLA model): unit tests numpy only (seconds); I-tests FK only,
I8 ~1 min; E1 one sim process ~2-3 min; E2/E3 one 85 e2e run each, a few minutes; `run_tests.sh` once.

---

## 5. Open rulings for Finnegan

- **R-F1 margins** (rule in 1.2): table 1.3, stand 2.0, basket 0.9, self 2.0 cm. Alternative: 2.0 cm everywhere (more
  protection; refuses approach in 17 of 50 demo-like motions). With the layout error, no margin promises "no touch" (0).
- **R-F2 streak**: one chunk + 0.5 s of refused ticks (109 at 0.5, 192 at 0.25). Alternative: fixed 1 s.
- **R-F3 other arm**: min(cmd, pred) like every group (teacher 5.8, demos 5.3 cm, VLA 5.3 cm on cmd). Recommend as proposed.
- **R-F4 operator display**: as 1.7.
- ~~R-F5 shadow abort~~: dropped (M4).
- **R-F6 blend**: 5 actions in `run` on 86 and 82; never in replay.
- **R-F7 layout entry**: sheet numbers in the robot's frame (1.3), refuse without them off loopback.
- **R-F8 replay stand placement (new)**: for the 86 replay, put the stand with the ball **6.7 cm further back and 1.7 cm
  toward the centre** (where the demo had it relative to the robot) and enter it. Fence then 3.54 cm from the stand instead
  of 1.70 cm; without it the replay passes ~1.7 cm (model) from the real stand and the fence would refuse there.
- **R-F9 robot laptop (new)**: the fence needs `mujoco==3.13.0`, which has no Intel-Mac wheel. Which laptops do the
  operators bring? An Intel Mac cannot run 86 with the fence; Apple Silicon or Ubuntu x86_64 can.
- **R-F10 IMU tilt (new)**: pelvis roll/pitch from the LowState IMU in `pred` (proposed), or the sim's fixed crouch tilt.
  Needs the 84 bring-up to confirm the IMU reads a few degrees at rest.

Adjacent finding (not fixed, flagged): 18 of 100 held-out teacher trials command the wrist pitch (dims 5 and 12) above
`ENVELOPE_HI` (`real_safety.py:47-50`) by up to 0.12 rad for up to 102 actions in a row; at 86 timing that passes
`ENVELOPE_STREAK_TICKS = 30`, so a correct v2 policy would be clipped and soft-exited. Cause not checked; refresh from the v2
stats (ruling).

---

## 6. Built (implementer, 2026-11-08)

Mac only, one sim process at a time, no VLA model loaded. Raw output: `/tmp/fence/build/`, `/tmp/fence/e1/`,
`/tmp/fence/tests/`, `/tmp/fence/mutants/`, `/tmp/fence/run_tests/`, `/tmp/fence/e2e/`.

### 6.1 Files

| Step | Files |
|---|---|
| B0 | new `tests/helpers/make_fence_model.py` -> `g1sim/data/fence_model.xml.gz` (0.54 MB gzip; hull vs full model 0.0009 mm over 60 poses, every group); new `tests/helpers/make_replay_demo_layout.py` -> `tests/data/replay_demo_layout.json` (head image 3.76; demo stand vs nominal in the robot frame +6.7 cm forward, -1.7 cm left = R-F8) |
| B1 | new `g1sim/fence.py`: `Fence` (from_env, nominal, set_layout, pose_values, values = min(cmd, pred) + midpoint, held_values, measured_values, check), `refused_groups` (never-closer rule), `low_groups`, `layout_from_sheet`, `FenceUnavailable`, pinned `MUJOCO_VERSION`, `DATA_FILES`; mujoco imported optionally (86 refuses instead of crashing) |
| B2 | `g1sim/real_safety.py`: `SafetyFilter(..., fence, fence_streak_ticks)`, `step(..., hands12, pelvis_rp)`, fence stage after the rate limit and before the lead monitor, `fence_hold` / `fence_streak`, `FenceLog` (smallest value, holds, hold/send alternations N5, overshoot over 1 s, fence time), `fence_streak_ticks(speed)` |
| B3 | `g1sim/real_run.py` (RunIO.fence, RunArgs.fence_layout/no_fence, IMU roll/pitch per tick, hands commanded only without `fence_hold`, `fence_streak` -> soft exit, one `FENCE:` line per streak), `g1sim/real_run_policy.py` (`_target` returns hands, shadow cmd-only count, `FENCE SUMMARY` line + sidecar `fence`), `g1sim/real_io.py` (`imu_roll_pitch`) |
| B4 | `scripts/86_real_run.py` (`--fence-setback-cm`, `--fence-table-dz-cm`, `--fence-stand-dx/dy-cm`, `--fence-basket-dx/dy-cm`, `--fence-no-basket`, `--fence-robot-yaw-deg`, `--fence-layout`, `--no-fence`; fence built before the gate), `g1sim/real_preflight.py` (`fence_option_problem`: replay/run need a layout, `--no-fence` loopback only, no silent default on loopback (N3); `fence_problem`: fail closed), `g1sim/motion_gate.py` (`loaded_repo_files` adds a loaded module's `DATA_FILES`) |
| B5 | `g1sim/real_timing.py`: `ChunkScheduler(blend=)`, `BLEND_ACTIONS = 5`; only `run` uses it |
| B6 | `g1sim/vla_eval.py` (`FenceGuard`, `--blend` fade, row fields `fence_holds/first_tick/worst_cm/exit/alternations`, `smooth_p99_acc`, summary `fence`; the teacher's `env.apply` is wrapped per trial, `ball_grasp.py` unchanged), `scripts/82_vla_eval.py` (`--fence`, `--blend N`) |
| B7 | `docs/ROBOT_DAY_RUNBOOK.md`: M3 lines get `--fence-setback-cm 100` (table away), M4 `<FENCE-REPLAY>` + stand placement R-F8, fence go/no-go after M4 (N4: any hold, or a value > 1 cm below table 2.31 / stand 3.54 / basket 3.02), M5/M6 `<FENCE>`, box "Fence layout" (what the operator sees) |
| B8 | `scripts/setup_robot_laptop.sh` (`mujoco==3.13.0` into the robot env; Intel Mac: FAILED line, 84/89 still work), `scripts/89_robot_day_selftest.py` (CHECK fence: version, model load, p99 of the fence check over the 725 replay-demo messages < 5 ms) |
| tests | new `tests/test_unit_fence.py` (17), `tests/test_unit_real_run_fence.py` (15), `tests/test_integration_fence.py` (11), `tests/data/run1_45k_chunks.npz` (S1); changed `tests/helpers/fake_real_run.py` (IMU field, loopback `no_fence` default), `tests/test_unit_motion_gate.py` (build_io signature), `tests/test_e2e_real_adapter_sim.py` (E2 `--fence-layout` + 0 holds + p99, new E3, `--no-fence` on the 40k runs) |

Proposed values behind named constants with "ruling pending (Finnegan)": `fence.MARGIN_M` / `SELF_MARGIN_M` /
`DISTRACTOR_MARGIN_M` (R-F1), `fence.PELVIS_TILT_FROM_IMU` (R-F10), `real_safety.FENCE_STREAK_EXTRA_S` and
`vla_eval.FENCE_STREAK_EXTRA_ACTIONS` (R-F2), `real_timing.BLEND_ACTIONS` (R-F6).

Decisions taken while building: N1 midpoint check as proposed (scene group < 3 cm -> also the halfway pose; U9 replaced by
the midpoint unit test); N2 F(hold) uses the last sent hand target; N3 loopback without layout is refused (`--no-fence`
needed); N5 alternations counted and reported (no sliding window yet); N6 basket margin 0.9 = 0.88 rounded to 1 mm. I4
asserts scene geoms within 0.01 mm and fence values within 1 mm (not 0.01 mm): the sim's settled crouch under the pelvis weld
(legs 0.005 rad, pelvis 0.04 deg) differs from the nominal crouch by up to 0.8 mm. I6 uses +-1 mm for the same reason.

### 6.2 Measured

| check | result |
|---|---|
| E1 `82 --teacher --fence --no-walkup --seed 0 --trials 50` (v2, 92 s, one process) | **50/50 passed, 0 holds, 0 fence exits**; same as without the fence (runs/vla_eval_teacher_seed0.json 50/50, /tmp/fence/teacher50 50/50). Smallest: table 4.43, stand 3.70, basket 3.60, torso 4.04, other arm 5.76 cm (plan: 4.46/3.70/3.60/4.04/5.76). No distractors in the walk scene model, so none fenced. Report `runs/vla_eval_fence_e1_teacher_s0.json` |
| replay demo, own layout, hull model (I6) | 0 refused; table 2.04, stand 3.33, basket 4.23 cm (sim's own model: 1.97/3.27/4.18) |
| replay demo, nominal + R-F8 / nominal (I6b) | 0 refused / 37 actions below the stand margin (stand 1.71 cm) |
| fence check time, Mac M4 | 89 CHECK fence on the 725 replay messages: p99 1.34 ms; model load 0.05 s; 85 e2e run: p99 4.17 ms (many holds, 4-6 FK each) |
| S1 run1 45k chunks at 0.5 | above 17.1 without the fade, below with 5 (latency 0 and 3) |
| mutants | 26 applied, 26 caught (`/tmp/fence/mutants/mutants*.log`); 20 and 22 not applicable: the fence has no measured-hand input at all |
| `scripts/run_tests.sh` (once) | project env 318 passed; sim env 571 passed + 7 errors/failures in `test_integration_fence.py` from test order (another test rebuilds the sim folder and deletes the walk scene) - fixed with a scene fixture, file re-run alone: 11 passed; openpi env 10 passed |
| 85 e2e (once, 7.5 min) | 7 passed, 2 skipped (40k), 2 failed: `test_ready...` = known L1 (waist 0.0296 > 0.02); `test_replay_tracks_demo_inside_envelope` = **E2 fails, see 6.3**. New E3 passed |

### 6.3 E2 finding (not tuned, for Finnegan)

In 85 the replay demo with its own layout is held: `FENCE SUMMARY holds 302 of 712 | smallest cm: basket -0.39, stand -0.96,
table 6.34 | overshoot cm: basket 0.45, stand 2.54`, then `SOFT EXIT fence_streak`. Offline from the run's log (no IMU, so the
crouch tilt; `/tmp/fence/e2e/analyse.log`): the commanded poses stay >= 7.1 cm from the basket and 9.5 cm from the stand (0
messages below a margin on cmd alone); the measured arm comes to 0.14 cm (basket) and 1.63 cm (stand). Cause: in 85 the torso
leans forward much more than in the training sim - measured waist pitch 0.069-0.111 rad vs 0.047 rad when the demo was
recorded (the same sag as review C L1) - and the arm tracks worse (max lead 0.249 rad). The fence sees that arm correctly:
in 85 the replayed arm really passes within millimetres of where the basket and stand would be. Options (rulings, not taken):
(a) accept that the 85 replay with a fence ends in fence_streak and gate E2 on the fence's numbers instead of exit 0;
(b) hold the waist pitch harder (gains) so the torso sag matches the sim; (c) check the robot's real waist sag in M2 first -
if it is like 85's, the M4 replay at the table will be held too, and the runbook's go/no-go (6.1 B7) stops there.
The 85 IMU tilt was not logged, so its share is not known; the offline numbers without it already show the holds.

### 6.4 Not done

- I8 (margin rule on the shipped checkpoint's dataset): needs the v1 dataset demos in their own layouts; the rule was applied
  by hand from `/tmp/fence/b2/demos50_fence.json`. Must be built before run3 ships.
- S2/S3 (82 `--fence --blend` with a VLA; run3 with and without): need the model, not on the Mac.
- E4 on the robot laptop: 89 CHECK fence does it there; on this Mac only.
- R-F1..R-F10 rulings; the motion lock (`docs/ROBOT_DAY_MOTION.json`) must be re-unlocked by Finnegan at the commit with
  this code and the model file.

### 6.5 O2 fixed: the way out after a fence stop vs the robot's own body (2026-11-10)

Finding (code review `.planning/reviews/fence-smooth.md` O2): after a fence stop near the torso or head, the soft exit's lift
leg (elbow 1.0 rad more bent) folds the hand up into the chest or face; 87 checks the way out only against table, stand and
basket. Same for a fault recover from a policy-phase pose (`arm_paths.home` -> `way_out`).

Fix (smallest change, the fence's own geometry, so 82/86 and the hull model agree):
- `Fence.route_refused(poses, hands12, measured17, pelvis_rp)`: the policy phase's never-closer rule along a whole route -
  refused if any group (table, stand, basket, torso, head, pelvis/legs, other arm) comes below its margin AND closer than at
  the route's start (0.1 mm tolerance); waist roll/pitch measured, hands at their last command; NaN refuses.
- `arm_paths.way_out(..., route)`: `lift` (unchanged, 87's lift_home rows) or `straight` (one joint line to the ready pose,
  the way the policy left it); `arm_paths.ready_index`.
- `real_run_exits.safe_way_out`: before a soft exit from the policy phase and before a fault recover from policy/hold/
  shadow, check the 50 Hz targets up to the ready pose: lift if it passes, else straight, else **no movement**: soft exit ->
  fault `exit_route_into_body` (`FAULT_ACTIONS`: HOLD), recover -> fault hold. Arms stay at weight 1; never dropped. The part
  after the ready pose (way in backwards) is unchanged. Without a fence (modes with no policy phase) nothing changes.
- The check costs 35-115 ms of FK once (review O2 m2: up to 252 ms on the M4, median 58, p99 113) before the exit starts (no messages in that time; `Runner.wait` restarts the 50 Hz
  schedule, no burst).

Measured (hull model, nominal layout, ready hands; the reviewer's 10 held poses `/tmp/fence/review3/probe_exit.log`, log
`/tmp/fence-o2/measure/before_after.log`): closest self distance on the way to the ready pose

| held pose | start | old lift route | new route |
|---|---|---|---|
| torso L fingertip / L palm / R fingertip / R palm | 2.11 / 2.60 / 2.65 / 2.10 | -5.97 / -4.65 / -0.10 / -5.38 | straight: 2.11 / 2.60 / 2.65 / 2.10 |
| head L fingertip / L palm / L forearm | 2.66 / 2.19 / 6.24 | 2.26 / -0.02 / 3.00 | lift 2.26 / straight 2.19 / lift 3.00 |
| head R fingertip / R palm / R forearm | 2.48 / 2.49 / 2.17 | -2.19 / -5.40 / -3.07 | straight: 2.48 / 2.49 / 2.17 |
| ready pose | 4.56 | 4.55 | lift 4.55 |

(cm, smallest of torso, head, pelvis/legs, other arm.) No route went into the body; none needed the hold. Tests:
`tests/test_integration_fence.py` (3 held poses: old route > 2 cm inside, new route never closer; lift kept where it is
clear) and `tests/test_unit_real_run_fence.py` (`CageFence`: every route refused -> fault hold, no to_ready; fault recover
from a policy pose -> hold, no to_factory). 5 of the 6 new tests fail on the old code (the 6th pins the unchanged lift route).

Not covered: the straight route is not in 87's 5 cm table rows (the fence checks it against table/stand/basket at the fence
margins with the measured layout); the way in backwards (ready -> factory) is still not checked against the body (it starts
at the ready pose, not at a policy pose); the motion lock must be re-unlocked by Finnegan for the changed modules.
