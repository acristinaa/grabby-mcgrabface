# Review: level1-walkup PLAN.md, lens SIM + CONTROL FEASIBILITY (plan review, pre-build)

Reviewer: independent gate (did not author the plan, no author reasoning seen). Date 2026-10-08.
Plan: `.planning/level1-walkup/PLAN.md`. Fixed user decisions were not challenged.

## Verdicts
- (1) Internally sound and buildable as written: **NO**. Two geometry problems (B1, B2) and two control problems (M3, M4) mean the
  Stage 3 and Stage 4 done criteria cannot be met as specified.
- (2) Right plan for the goal: **YES in architecture**: synchronous GR00T walk-up, weld at the handover, episodes starting at the handover,
  reuse of the validated teacher, and the replay check. Sections A/B/C and the Stage 3/4 criteria need the corrections below.
- Overall: **NO-GO**. The fixes stay inside A/B/C and Stages 3/4. Re-review after they are folded in.

## How the evidence was produced (scripts in the session scratchpad, all read-only against the repo)
- `fk.py`, `fk2.py`, `fk3.py`: forward kinematics on `~/dev/g1-ext/g1_brainco_sim/assets/scene_29dof_brainco_volleyball.xml`,
  with the pelvis moved through `body_pos` and `mj_geomDistance` to `table_top`.
- `stand.py`: P3's synchronous GR00T loop (the same xfrc band, the LeRobot gains) on `/private/tmp/claude-501/walkprobe/scene_walk_volleyball.xml`,
  3 s band + 4 s free stand, at dt 0.004 and dt 1/300, once with READY arms and once with the planned hanging start arms.
- `teach.py`, `teach75.py`: the current teacher (`TwoHandBallGrasp`) on TableEnv("volleyball") without images. The second run
  lowers the pelvis to 0.751 and puts the legs in GR00T's measured crouch.
- `mc.py`: Monte Carlo of the stance envelope.

## Findings

### B1 (blocker): the knees hit the solid table inside the plan's own layout and envelope ranges
- Trigger: GR00T's real standing posture is a deep crouch. `stand.py` measured pelvis z 0.7505-0.7517, knee 0.68-0.72 rad and hip pitch
  -0.30..-0.43 at both timesteps and with both arm poses. The table is a solid box from the floor to the top (`table_scene.py:132-138`,
  size z = TABLE_TOP/2, centred at TABLE_TOP/2).
- Evidence (`fk3.py`, crouch posture, pelvis z 0.751): distance from `right_knee_link` to `table_top` by pelvis-to-front-edge distance:

  | Pelvis to edge (m) | Knee to table (m) |
  |---|---|
  | 0.20 (nominal) | +0.019 |
  | 0.17 (plan A layout minimum) | -0.011 |
  | 0.13 (plan B envelope minimum) | -0.051 |

  The plan's "the legs hit the solid sim table below about 0.12 m" (A, line 73) only holds for straight legs.
- Impact:
  - `walkup_contact` fires.
  - `ball_grasp.FORBIDDEN` includes "table" and `robot_contacts()` counts every robot body, so knee contact also fails
    `no_forbidden_contacts` during the grasp.
  - The Stage 4 criterion "no walk-up contacts ≥ 48/50" cannot be reached. The nominal stop has only 1.9 cm of margin.
- Fix (cheaper one first):
  1. Give the `volleyball_walk` table an open front: a top slab about 4 cm thick on legs or a narrow apron. Real tables have this (U5).
  2. Or move the nominal stop to at least 0.24 m from the edge and re-check reach.
  
  Either way, derive the A/B edge limits by FK with the measured GR00T posture, not by hand.

### B2 (blocker for Stage 3 as written): the arms collide with the table, both in the hanging start pose and on the way up to READY
- Start pose `[0.2,0,0,1.0,0,0,0]`: the fingers hang about 0.18 m ahead of the pelvis at z 0.48-0.67, below the table top (0.74).
  Thumb-tip clearance to the table (`fk.py`), pelvis z 0.75:

  | Pelvis to edge (m) | Clearance (m) |
  |---|---|
  | 0.20 | +0.024 |
  | 0.17 | -0.006 |
  | 0.13 | -0.046 |

  So Stage 3's own "≥ 0.03 m" rule fails even at the nominal stop. The suggested fallback, shoulder pitch +0.3, still gives -0.005 at 0.13.
- `raise` phase, "elbow first, then shoulder" (`fk2.py`): the fingertips sweep forward around the elbow and go 0.088-0.148 m into the table.
  This happens at every pelvis-edge distance from 0.13 to 0.20 and at both pelvis heights (0.75 and 0.785). A plain linear joint path
  is the same (-0.098 to -0.154). The claim "the hands rise close to the body and stay behind the table edge" (C, line 124) is false
  for this kinematics.
- Impact: every Stage 3 sweep trial fails `no_forbidden_contacts`, and in walk mode the hanging arms touch the table on arrival.
- Fix:
  - Raise the arms to the carry pose before the final approach, at least 0.4 m from the edge, during stand/look.
    P3 already walked fine with READY arms (`walkprobe/p3.log`: arrived, no fall).
  - Note that READY hands have their lowest geom at z 1.04 when the pelvis is at 0.75. The ball top is at 1.017, so there is only
    about 2 cm of vertical clearance while arriving. Add hand-to-ball and hand-to-basket clearance to the check.
  - Alternatively, keep a raise at the table, but only along a path that an FK sweep over the whole envelope proves clear,
    as a unit test in Stage 3.

### M3 (major): the STAND_Z / leg posture spec contradicts itself
- Plan A, lines 60-62: legs at GR00T's default angles (hip -0.1, knee 0.3, ankle -0.2) and "STAND_Z computed by FK, expected about 0.75".
- FK with those angles puts the feet on the floor at pelvis z **0.784** (`fk2.py`; straight legs give 0.792).
  GR00T really stands at 0.751 with knee about 0.7 (`stand.py`, matching P1-P3 at 0.746-0.751).
- Impact:
  - In no-walk mode the robot would stand about 3.3 cm higher than at a walk-mode handover, so the camera and shoulders differ between
    the recording fallback and the eval. This breaks "Episode frames are identical in both modes" and R8.
  - The weld "band" would hold the pelvis at 0.784 while GR00T commands a 0.74 height.
  - The Stage 4 criterion "handover pelvis z within 0.01 m of STAND_Z" fails by about 3 cm.
- Good news: the teacher still passes the nominal layout at pelvis 0.751 with the crouched legs (`teach75.py`: passed, lift 0.0874 m,
  ball 0.8 mm off the basket centre after the carry). The height itself is not a problem.
- Fix: take STAND_Z and the 12 leg angles plus waist from a measured GR00T Balance settle. Store them as constants with their source,
  and use them for no-walk mode, the band and the criterion.

### M4 (major): arrival precision cannot meet the envelope, and the re-approach does nothing
- `GotoParams.pos_tol = 0.10`, `yaw_tol = 10 deg` (`navigation.py:64-65`). `step()` returns zero as soon as the robot is inside the
  10 cm circle (`navigation.py:90-91`). So the robot stops short of the goal along its approach direction, every time:
  - P2: 8.1 cm short, pelvis 0.27 m from the edge for a 0.20 m goal;
  - P3: 9.3 cm short.
- `mc.py`: robot starts 0.5-1.0 m back and ±0.25 m to the side, with the plan's goal perturbation and an arrival 3.5-9.3 cm short.
  Only **about 50 %** of stances land inside the envelope (r 0.34-0.50 for both objects, pelvis ≥ 0.13 m from the edge).
  The p95 of the larger r is 0.548. Without the perturbation it is 96 %.
- The single re-approach to the unperturbed goal does nothing when the stop is already within 0.10 m of that goal, because `arrived()`
  is immediately true. So it cannot rescue the typical case.
- The Stage 4 criterion "≥ 48/50 inside the envelope" is not reachable as written.
- No-walk mode simulates a symmetric ±6 cm arrival error. The real error is biased, so the recording fallback distribution ≠ the eval
  distribution.
- Fix:
  - Add a fine-positioning end phase (for example pos_tol 0.02-0.03 m and 3 deg at `min_cmd` speed). This is untested with GR00T, so
    probe it in Stage 4 first.
  - Or drop the goal perturbation and let the natural arrival error be the variation.
  - In both cases, measure the arrival distribution from about 100 synchronous walk-ups (about 0.5 s each) and sample no-walk stances
    from that measured distribution.

### m5 (minor): the bearing part of the envelope is undefined
- "angle to the objects within the reach test" (B, line 108) has no numbers.
- The ±8 deg heading perturbation plus up to 10 deg of arrival yaw moves the 35-50 deg bearings by up to ±18 deg, so the waist would
  have to turn up to about 68 deg. The teacher has never run that. Its comment at line 31 notes wrist contact with the basket already
  at 30 deg.
- Fix: put numeric bearing and waist bounds, taken from the Stage 3 sweep, into `configs/randomization.json`.

### m6 (minor): switching the timestep is an avoidable failure mode
- GR00T Balance stands just as well at dt 1/300 with 6 substeps per 50 Hz tick (`stand.py`: pelvis 0.7505/0.7517, roll/pitch < 1.1 deg).
- If an exception during the walk-up leaves dt at 0.004, the next `TableEnv.apply` (SUBSTEPS = 10) runs 25 Hz ticks that are
  recorded as 30 Hz, and nothing reports it.
- Fix: one dt of 1/300 throughout, with GR00T every 6 substeps. Walking at 1/300 is not probed yet, so add that to Stage 4.
  If the switch stays, set dt unconditionally in `reset()`.

### m7 (minor): replaying through a re-run walk-up is fragile
- `82 --replay-variation` reaches the handover by running the walk-up again. Two things can make that run differ:
  - the look renders consume `noise_rng`;
  - `payload.set_head` writes `model.eq_data`, which `ModelVariation.apply` never restores (FIELDS, `randomization.py:119-120`),
    so a head tilt left over from the previous trial carries into the next one.
- A divergent handover makes the open-loop replay fail for no real reason.
- Fix: save `mj_getState` (integration state, plus mocap, `eq_active` and the head `eq_data`) at the handover in the sidecar, and restore
  it for the replay.

### m8 (minor): reset ordering with the weld, and one variation group that silently does nothing
- `mj_resetData` (`table_env.py:117`) resets `mocap_pos` to the pose from build time and `eq_active` to `eq_active0`.
  The stance and the mocap pose must both be written before the 0.3 s settle loop (`table_env.py:104-105`), or the weld pulls the
  pelvis back to the origin.
- The "stand" group of `ModelVariation` writes the pelvis `body_pos`/`body_quat` (`randomization.py:137-139`). With a free joint,
  MuJoCo ignores these, so the change silently has no effect but would still be logged in the sidecars. Turn the group off for
  `volleyball_walk`.

### m9 (minor): the effort estimate and the timeline
- Stage 1 (4-5 h) bundles the global 27-D change (three envs, openpi convert), the free base, the weld and the tests.
- Stages 3 and 4 now also carry the table, raise and fine-positioning redesign.
- Thursday holds Stages 3, 4, 5, 6, 7 and 8. That is not realistic.
- The realistic route is the plan's own no-walk recording fallback. That route only becomes valid after M3 and M4 have fixed the
  posture and the stance distribution.

## Claims checked and confirmed
- Waist-yaw `xanchor` = pelvis origin ([0, 0, z]), so the nominal grasp waist is atan2(0.241, 0.344) = 35.02 deg, as the plan says.
- The gains are equal (`config_unitree_g1.py:25-33` and `table_env.py:33-34`). GR00T writes 15 targets (`gr00t_locomotion.py:199-205`)
  and holds `groot_height_cmd` = 0.74.
- The synchronous GR00T loop works (P3 log: arrived, 9.3 cm, 31x real time). It also stands with the hanging start arms.
- Teacher speed without images: 1.32 s wall time for 27.5 s of sim. That makes the "< 5 s per trial walk + teach" and "200-trial sweep
  in 5-10 min" estimates plausible.
- Geometry constants: basket rim z = 0.74 + 0.005 + 0.10 = 0.845; ball rest z = 0.912.

## Not verifiable, so these items keep the gate OPEN until Stage 3 measures them again
- The grasp-reach probe results cited in A and C: "teacher passed r 0.34-0.50", "45 deg failed, 70 deg 60/60", "mirrored layouts passed",
  "align needed, up to 4.8 cm". There are no scripts or logs for them on disk.
  `/private/tmp/claude-501/probe` holds only the camera and tilt probes; `walkprobe` holds only P1-P3.
- Walking (as opposed to standing) at dt 1/300, and walking with the hanging start arms: not run.
- Perception accuracy of the sphere fit and the rim bounding box from 0.5-1.0 m back with heading ±30 deg: not run.
