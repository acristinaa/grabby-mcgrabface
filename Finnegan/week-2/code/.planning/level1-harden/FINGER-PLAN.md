---
phase: level1-harden
plan: finger-clearance
revision: 1 (2026-10-27, planner; measured, no code changed)
depends_on:
  - SmolVLA run1 held-out eval 2026-10-16 (runs/vla_eval_run1_040k_seed0.json, runs/vla_eval_run1_045k_seed0.json)  # done
  - teacher state of commit 450fd4b (shoulder lift, walk sweep 200/200)                                            # done
files_modified:
  # Task 1 owns
  - g1sim/hand_clearance.py                      # new: per-tick distance meter, arm parts vs forbidden bodies, pooled summary
  - scripts/67_volleyball_pick_place.py          # additive: --clearance flag, per-trial "clearance" block, report summary + 1 line
  - tests/test_integration_hand_clearance.py     # new: 2 tests
  # Task 2 owns
  - g1sim/ball_grasp.py                          # 3 constants + their comments + 1 docstring line
  - docs/volleyball_teacher_squeeze.png          # rewritten by the 67 fixed-scene nominal run (new cradle picture), expected
  # integration step (main session) owns, after the gate
  - PROGRESS.md, docs/LEARNING_PATH.md (§6, §9, §10), COMMANDS.md ("Teacher clearance" block)
---

# Level 1 hardening: little-finger clearance of the sim teacher

Repo root (all paths are relative to it):
`/Users/finneganadam/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot`.
`PY=~/dev/g1-ext/.venv/bin/python` (sim env, mujoco 3.13.0). Every run writes its full output to `runs/l1h_finger_*.{json,log}`;
read the verdict from the file, never through `head`/`tail`.
Scope: the teacher (`g1sim/ball_grasp.py`) and a measurement tool for it. No VLA training, no demo recording (T7 not mounted), no
change to `table_env.py`, `episode.py`, 69, 79-87, the real-robot files or `tests/data/*`. No commit or push by build agents.

## 1. Evidence (measured 2026-10-27, read from files and re-run in the sim)

### 1.1 What the student fails on (run1 held-out seed 0, 50 starts, from the eval JSON rows)

| Checkpoint | Pass | Forbidden contact | Ball not lifted | Other |
|---|---|---|---|---|
| 40k | 36 | 6: L pinky-stand 2, L pinky-basket 3, L pinky-table 1 | 7 | 1 placement |
| 45k | 35 | 12: L pinky-stand 6, R pinky-stand 1, L/R pinky-table 2, R pinky-basket 1, R palm-basket 1, L pinky-basket 1 | 3 | 0 |

Sim time of the contact vs. the teacher's phase clock (phase start s: face_ball 0, reach 2.0, touch 6.0, squeeze 8.5, lift 10.0,
turn 13.4, centre 17.9, release 18.9):
- table: 4.4 s in all 3 cases = **the last ticks of the reach**.
- stand: 3.9 s (1 case, reach), 8.9-12.3 s (7 cases) = **touch / squeeze / lift**, 2 later (15.4 s, 24.7 s).
- basket: 16.5-33 s; in every basket case the `lifted` event never fired, so these hands were moving without the ball (a
  consequence of a failed grasp, not a clearance problem of the carry; see section 8).
All contact parts are the pinky (distal link or tip) except one right palm.

Correction to the task brief: "ring + pinky open since D2" is the **bottle** grasp (`g1sim/grasp.py`). The ball teacher curls all
five fingers (`FINGERS_CURLED`, ring and pinky 0.12, `ball_grasp.py:86`) and opens all of them to 0 during the reach (`:207`).

### 1.2 How close the teacher passes (new measurement)

Probe (planner scratch, kept with all results in `/tmp/l1day1/probe/finger_clearance_probe.py`; outputs `/tmp/l1day1/fc_s*.json`,
summaries `/tmp/l1day1/fc_analysis*.txt`, `per_seed.txt`): the unchanged teacher on exactly 69/82's no-walk starts
(`TableEnv("volleyball_walk", seed=S, randomization.load())`, one `reset(randomize=True)` per trial; seed 0 arrival offsets equal
the 82 eval rows in 50 of 50). After every control tick: `mujoco.mj_geomDistance` (distmax 0.15 m) from every collision geom of
the elbow, wrists, palm and fingers of both arms to every geom of `table`, `volleyball_stand`, `basket`. 600 starts: held-out
seeds 0+1 (100), sweep seeds 10-13 (200, the 10-14 "200/200" sweep), training seeds 101-104 x 75 (300 = the v1 demo starts).
Teacher result: 599/600 (the one failure, s104_t0072, is a known start where the ball leaves its stand before the touch: moved
17 cm, no robot contact before `touch`; v1 recorded 299/300 too).

Smallest distance per part, passed trials (cm):

| Part vs body | held-out 0+1: min / p5 / p50 | sweep 10-13: min / p5 / p50 | phase of the minimum |
|---|---|---|---|
| L pinky - table | 0.48 / 0.70 / 2.47 | 0.58 / 0.95 / 2.31 | reach, t = 4.43-4.53 s (100 %) |
| R pinky - table | 0.63 / 0.86 / 2.29 | 0.24 / 0.86 / 2.14 | reach, same ticks |
| R wrist - basket | 1.02 / 1.73 / 3.65 | 0.05 / 1.13 / 3.50 | reach |
| L / R pinky - stand | 2.31 / 2.36 / 2.52 | 2.30 / 2.37 / 2.52 | touch (~70 %), squeeze (~30 %) |
| L / R palm - stand | 4.28 / 4.30 / 4.4 | 4.24 / 4.30 / 4.4 | touch / squeeze |
| any arm part, any body | 0.48 / 0.69 / 2.05 | 0.05 / 0.67 / 1.85 | |

- **Pinky - table** is a dip at the end of the reach: the distance is purely vertical (nearest points differ only in z), the pinky
  distal is ~2 cm above the top (0.763 vs 0.740 m) and it rises again during the reach settle and touch (touch min 8.6 cm). It is
  worse when the robot stands further back (correlation with `arrival.back_m` -0.75, with `side_m` +0.51).
- **Pinky - stand** is nearly constant (2.3-2.5 cm in every start) for the ~4 s of touch + squeeze: the pinky tip sits ~1.1 cm above
  the stand's top rim and ~2.2 cm outside it. It is geometry, not randomization.

## 2. Root cause

The teacher's path has no margin where the pinky is lowest:
1. **End of the reach (t = 4.5 s):** reach goal = touch pose + 4.5 cm out along the palm normal + `REACH_ABOVE` 4 cm up. With the
   palm 20 degrees below the equator the "out" direction points 1.5 cm down, the touch pose itself is r sin 20 = 3.6 cm below the
   ball centre, and the arm sags under gravity before `_settle("reach")` removes the sag. Result: the open pinky passes 0.7-0.9 cm
   (p5) above the table.
2. **Touch + squeeze (6-10 s):** with the palm 20 degrees below the equator and the fingers pointing 15 degrees down, the pinky is the
   lowest finger and its tip stays 2.3-2.5 cm from the stand rim for 4 s.
The teacher itself never touches (its closed-loop sag fix lands exactly), but the student copies a path whose margin (0.7 cm and
2.4 cm) is below its own closed-loop error. Estimate of that error: open-loop MAE at 40k is 0.006-0.008 rad on the arms and
0.015-0.02 rad on the waist (`runs/l1w_s8_open_loop_all.log`, training frames); 0.02 rad of waist at 0.42 m is 0.8 cm alone, held-out
closed-loop errors are larger. Stand contacts at a 2.4 cm margin in 4-14 % of held-out trials (40k 2/50, 45k 7/50) fit a
half-normal per-trial deviation with sigma ~1.2-1.6 cm.

## 3. Options measured (seeds 0 + 10, 100 walk-world starts each; fixed scene 67 `--randomize --seed 0` 50 starts)

Overrides of `ball_grasp` constants in a scratch runner (`/tmp/l1day1/probe/run67_variant.py`, results `/tmp/l1day1/fcv_*`,
`fch_*`, `f67_*`, summaries `fcv_compare*.txt`, `fch_summary.txt`, `final_compare.txt`):

| Variant | walk pass | any-part p5 / min (cm) | pinky-stand p5 | R wrist-basket p5 | slip p95 (mm) | fixed rand0 |
|---|---|---|---|---|---|---|
| baseline (CRADLE 20, REACH_ABOVE 0.04, SQUEEZE 0.025) | 100/100 | 0.69 / 0.05 | 2.34 | 1.07 | 6.2 | 50/50 |
| REACH_ABOVE 0.06 | 100/100 | 2.01 / 1.73 | 2.03 | 2.65 | - | 50/50 |
| REACH_ABOVE 0.08 | 100/100 | 1.66 / 1.57 | 1.66 | 4.37 | - | - |
| FINGER_TILT -5 (fingers less down) | **92/100** | -0.00 / -0.02 | 3.05 | **-0.00 (contact)** | - | - |
| FINGER_TILT 0 | **81/100** | -0.03 | 3.40 | contact | - | - |
| CRADLE 15 | 100/100 | 1.48 / 1.22 | 3.20 | 2.14 | - | **49/50** (lifted 5.7 cm) |
| CRADLE 15 + REACH_ABOVE 0.06 | 100/100 | 2.86 / 2.80 | 2.88 | 3.86 | 6.8 | 50/50 |
| CRADLE 12 + REACH_ABOVE 0.06 | 100/100 | 3.37 / 3.29 | 3.38 | 4.54 | 7.0 | **49/50** (carried) |
| CRADLE 10 + REACH_ABOVE 0.06 | 100/100 | 3.68 / 3.47 | 3.72 | 4.99 | 7.1 | **49/50** (carried) |
| **CRADLE 10 + REACH_ABOVE 0.06 + SQUEEZE 0.03** | 100/100 | **3.65 / 3.47** | **3.70** | 4.99 | 7.5 | **50/50** |

Why each lever works or not:
- `REACH_ABOVE` alone lifts the reach end (table +1.6 cm) but **lowers** the stand margin: the sag measured at the higher reach pose
  under-corrects the lower touch pose. Not enough alone.
- Rolling the fingers up (`FINGER_TILT_DEG`) helps the stand but drops the wrist onto the basket rim: rejected.
- `CRADLE_DEG` smaller raises both hands at touch (r sin 20 - r sin 10 = 1.8 cm) and makes "out" less downward: helps table and stand.
  Below 15 degrees it loses one extreme fixed-scene start (rand0 trial 37: stand -5.9/+9.7 cm, ball +8.8/+2.3 cm; the ball sinks in
  the turn), which a 0.5 cm deeper squeeze restores (the 10-07 probe held 1.5-4 cm squeezes). The squeeze does not change clearance.
- `APPROACH_OUT` larger would push the reach end further down-out (wrong direction); phase timing does not move the minima (they
  are at the end poses, not mid-path). Not pursued.

## 4. The change (Task 2): three constants in `g1sim/ball_grasp.py`

| Constant | Now | New | Effect (measured) |
|---|---|---|---|
| `CRADLE_DEG` | 20.0 | **10.0** | palms 10 degrees below the equator: hands ~1.8 cm higher at touch |
| `REACH_ABOVE` | 0.04 | **0.06** | reach end 2 cm higher: removes the table dip of the sagging arm |
| `SQUEEZE` | 0.025 | **0.03** | 0.5 cm firmer hold: keeps the extreme fixed-scene start (rand0 #37) |

Each comment states the new value's source (this plan + the `runs/l1h_finger_*` files) and keeps the old history line short:
CRADLE_DEG "20 held 36/36 in the 10-07 probe, equator 8/36; 10 chosen 2026-10-27 for pinky clearance (stand 2.4 -> 3.7 cm), held
with SQUEEZE 0.03 in 600 walk + 190 fixed-scene starts"; REACH_ABOVE "0.02 hit the basket rim, 0.04 left the pinky 0.7 cm (p5) above
the table at the reach end, 0.06 -> 3.7 cm with CRADLE_DEG 10"; SQUEEZE "1.5-4 cm all held in the probe; 3 cm since 2026-10-27".
Module docstring line 3 ("20 degrees below the equator") follows. No other line changes; `FINGERS_CURLED`, `FINGER_TILT_DEG`,
`APPROACH_OUT`, phase times stay.

## 5. Gate (all numbers on the changed code; deterministic sim, so the probe numbers must reproduce)

| # | Check | Command (section 6) | Pass | Probe value |
|---|---|---|---|---|
| G1 | fixed scene nominal | 67 `--trials 10` | 10/10, "ACCEPTANCE PASSED", exit 0 | 10/10 |
| G2 | fixed scene +-2 cm | 67 `--jitter 0.02 --seed 0` 50 | 50/50 | 50/50 |
| G3 | fixed scene randomized | 67 `--randomize` seed 0 and seed 1, 50 each | 50/50 and 50/50 | 50/50, 50/50 |
| G4 | fixed scene distractors | 67 `--jitter 0.02 --distractors --seed 0` 30 | 30/30 | 30/30 |
| G5 | walk sweep | 67 walk `--randomize --clearance` seeds 10-13 x 50 | >= 199/200 (was 200/200) | 200/200 |
| G6 | held-out starts | 67 walk `--randomize --clearance` seeds 0, 1 x 50 | 100/100 | 100/100 |
| G7 | **clearance** (pooled G5 + G6, passed trials, every arm part from the elbow down vs table/stand/basket) | `hand_clearance` pooled summary | **p5 >= 3.5 cm and min >= 3.0 cm; pinky p5 >= 3.5 cm** | p5 3.65-3.66, min 3.18, pinky p5 3.70-3.72 |
| G8 | teacher through the eval | 82 `--no-walkup --seed 0 --trials 50 --teacher` | 50/50, agree 50/50 | (G6 seed 0 = 50/50) |
| G9 | hold not weakened | G5 + G6 reports | lift min >= 8.0 cm (was 8.3) | 8.2 |
| G10 | tests | `scripts/run_tests.sh` (3 envs, not e2e) | no new failure **by name** vs the before-run; the 2 new tests pass | - |
| info | training starts (what v2 demos would see) | 67 walk `--randomize --clearance` seeds 101-104 x 75 | report only | 299/300, p5 3.63 |

Why X = 3.5 cm (p5) / 3.0 cm (min): with the sigma 1.2-1.6 cm estimated in section 2, a margin of 3.5 cm gives a per-location
contact chance of ~0.3-3 % (half-normal tail beyond 2.2-2.9 sigma) instead of 4-14 % at 2.4 cm and more at 0.7 cm; 3.0 cm as the
worst case keeps every start above 1.9 sigma. It is also the most the hold allows: CRADLE 10 is the smallest cradle measured and
already needs the deeper squeeze. The estimate rests on 2-7 contact events, so it is a working number, not a promise; the real
check is the student (run2 held-out eval, section 9). The day-plan check "0 finger contacts on the 200 sweep" is already true for
the old teacher (200/200) and so proves nothing; G7 replaces it (ruling R4).

## 6. Tasks

<task type="auto">
<name>Task 1 - Clearance meter for the teacher and 67 --clearance (built and checked on the OLD constants first)</name>
<files>g1sim/hand_clearance.py (new), scripts/67_volleyball_pick_place.py (additive), tests/test_integration_hand_clearance.py (new)</files>
<action>
- `g1sim/hand_clearance.py`, small functions, docstrings that say why (the student copies the teacher's path, so the teacher needs
  margin; contacts alone do not show near misses):
  - `DISTMAX_M = 0.15` (distances beyond it are reported as 0.15; the probe range).
  - `arm_parts(model)`: collision geoms (`contype or conaffinity`, as `TableEnv.robot_geoms`) of every body in the subtree of
    `left_elbow_link` / `right_elbow_link` (walk `model.body_parentid`), labelled `(side, part)` with `part = ball_grasp.finger_of(name)`
    for hand bodies (palm, thumb, index, middle, ring, pinky) and `"arm"` otherwise. Legs, torso, head are excluded on purpose (the
    knee is 4.1 cm from the table by design of the stop radius, `navigation`; it is not a reach problem).
  - `forbidden_geoms(model)`: per body name in `ball_grasp.FORBIDDEN` that exists in the model, its geoms.
  - `class ClearanceMeter(env)`: `update(phase)` after each tick: per `(side, part, body)` the smallest `mj_geomDistance`, and the
    overall closest `{side, part, body, d_m, phase, t_s}`; `result()` -> `{"min_m": {"left|pinky|table": d, ...}, "closest": {...},
    "min_by_phase_m": {phase: d}}` (rounded 4 decimals).
  - `summarize(trials)`: over trials that passed and have a "clearance" block: `n`, `p5_m`, `min_m`, `pinky_p5_m`, `pinky_min_m`,
    `closest_counts` (top 5 of "side|part|body|phase"). `numpy.percentile` default method (the probe used it).
- `scripts/67_volleyball_pick_place.py`: `--clearance` (help: "measure the smallest distance of every arm part to table, stand,
  basket; ~0.5 s more per trial"). When set: one `ClearanceMeter` per trial, called in the existing `on_tick` after the video grab;
  `result["clearance"] = meter.result()`; `report["clearance_summary"] = hand_clearance.summarize(results)`; one printed line
  `clearance (passed trials): any arm part p5 X cm, min Y cm; pinky p5 Z cm; closest most often: ...`. Without the flag, output
  is byte-for-byte as before (no new keys). Docstring gets one command line for the clearance sweep.
- `tests/test_integration_hand_clearance.py` (`pytest.mark.integration`, `needs_g1ext`, scene written like
  `test_integration_vla_eval.py`'s `scene` fixture), two tests, each with a "Proves / Catches" docstring:
  1. `test_meter_sees_a_pinky_touching_the_stand`: walk scene, `reset()`; shift `model.body_pos` of `volleyball_stand` so its geom
     `volleyball_stand_0` centre lands on the left pinky distal geom centre; `mj_forward`; `update("probe")`. Assert
     `min_m["left|pinky|volleyball_stand"] <= 0.001` and every labelled body is in an elbow subtree.
     Proves: the meter measures hand geoms against forbidden bodies with real distances. Catches: a geom filter that drops the hand,
     a clamp that hides contact, or one that lets legs/torso in.
  2. `test_teacher_keeps_the_margin_on_the_nominal_walk_start`: walk scene, `reset()` (nominal), `TwoHandBallGrasp(env,
     on_tick=meter.update).run()`; assert `passed` and `closest.d_m >= NOMINAL_MARGIN_M` with `NOMINAL_MARGIN_M = 0.035` (source:
     this plan G7; probe nominal: old constants 2.56 cm L pinky-stand at touch, new 4.06 cm).
     Proves: the teacher's nominal path keeps 3.5 cm from table/stand/basket with every arm part. Catches: a return to the old cradle
     or reach height (2.6 cm). **Must be seen failing on the old constants** (it is red until Task 2: log the failing run).
</action>
<verify>
cd "$REPO"; PY=~/dev/g1-ext/.venv/bin/python
$PY -m pytest tests/test_integration_hand_clearance.py -v > runs/l1h_finger_t1_tests_old.log 2>&1   # test 1 passes, test 2 FAILS (old constants)
$PY scripts/67_volleyball_pick_place.py --scene volleyball_walk --randomize --trials 50 --seed 10 --clearance --no-labview \
    --out runs/l1h_finger_old_walk_s10.json > runs/l1h_finger_old_walk_s10.log 2>&1
  # must reproduce the probe on seed 10 with the old constants: 50/50, any part p5 0.65 cm, min 0.05 cm, pinky p5 0.82 cm (+-0.01 cm;
  # a difference means the geom set differs from the probe's - fix before Task 2)
$PY scripts/67_volleyball_pick_place.py --scene volleyball_walk --randomize --trials 5 --seed 10 --no-labview \
    --out runs/l1h_finger_noflag.json > runs/l1h_finger_noflag.log 2>&1   # no "clearance" keys in the JSON
</verify>
<done>Meter + flag + 2 tests exist; test 1 green, test 2 red on the old constants (logged); the seed-10 old-constant numbers match the
probe within 0.01 cm; without --clearance the 67 report has no new keys.</done>
</task>

<task type="auto">
<name>Task 2 - Teacher margin: CRADLE_DEG 10, REACH_ABOVE 0.06, SQUEEZE 0.03, then the full gate</name>
<files>g1sim/ball_grasp.py, docs/volleyball_teacher_squeeze.png (rewritten by G1)</files>
<action>
- Before editing: `scripts/run_tests.sh > runs/l1h_finger_tests_before.log 2>&1` (baseline failures by name).
- Change exactly the three constants and their comments as in section 4, plus the docstring line. Nothing else.
- Run the gate commands below (independent runs may go in parallel, e.g. 6 at a time; each to its own file).
</action>
<verify>
cd "$REPO"; PY=~/dev/g1-ext/.venv/bin/python; W="--scene volleyball_walk --randomize --trials 50 --clearance --no-labview"
$PY scripts/67_volleyball_pick_place.py --trials 10 --no-labview --out runs/l1h_finger_fixed_nominal.json > runs/l1h_finger_fixed_nominal.log 2>&1   # G1
$PY scripts/67_volleyball_pick_place.py --trials 50 --jitter 0.02 --seed 0 --no-labview --out runs/l1h_finger_fixed_jitter.json > runs/l1h_finger_fixed_jitter.log 2>&1   # G2
for s in 0 1; do $PY scripts/67_volleyball_pick_place.py --trials 50 --randomize --seed $s --no-labview --out runs/l1h_finger_fixed_rand$s.json > runs/l1h_finger_fixed_rand$s.log 2>&1; done   # G3
$PY scripts/67_volleyball_pick_place.py --trials 30 --jitter 0.02 --distractors --seed 0 --no-labview --out runs/l1h_finger_fixed_distr.json > runs/l1h_finger_fixed_distr.log 2>&1   # G4
for s in 10 11 12 13 0 1; do $PY scripts/67_volleyball_pick_place.py $W --seed $s --out runs/l1h_finger_walk_s$s.json > runs/l1h_finger_walk_s$s.log 2>&1; done   # G5, G6
for s in 101 102 103 104; do $PY scripts/67_volleyball_pick_place.py --scene volleyball_walk --randomize --trials 75 --clearance --no-labview --seed $s --out runs/l1h_finger_walk_s$s.json > runs/l1h_finger_walk_s$s.log 2>&1; done   # info
$PY -c "import json,sys; sys.path.insert(0,'.'); from g1sim import hand_clearance as h; rows=[r for s in (10,11,12,13,0,1) for r in json.load(open(f'runs/l1h_finger_walk_s{s}.json'))['results']]; print(json.dumps(h.summarize(rows)))" > runs/l1h_finger_g7.log 2>&1   # G7 (and G9: lift min from the same rows)
$PY scripts/82_vla_eval.py --no-walkup --seed 0 --trials 50 --teacher --no-labview --name l1h_finger_teacher_seed0 > runs/l1h_finger_teacher_seed0.log 2>&1   # G8
$PY -m pytest tests/test_integration_hand_clearance.py -v > runs/l1h_finger_t2_tests_new.log 2>&1   # both green now
scripts/run_tests.sh > runs/l1h_finger_tests_after.log 2>&1   # G10
grep -E "^(FAILED|ERROR) " runs/l1h_finger_tests_before.log | sort > /tmp/l1day1/fail_before.txt
grep -E "^(FAILED|ERROR) " runs/l1h_finger_tests_after.log  | sort > /tmp/l1day1/fail_after.txt
diff /tmp/l1day1/fail_before.txt /tmp/l1day1/fail_after.txt > /tmp/l1day1/fail_diff.txt; echo "diff exit $?"   # 0 = no new failure by name
</verify>
<done>G1-G10 pass with the numbers of section 5 (expected to reproduce the probe: G7 p5 3.65-3.66 cm, min 3.18 cm, pinky p5 3.70-3.72 cm);
results table (gate, file, value) handed to the main session for PROGRESS/LEARNING_PATH; no commit by the build agent.</done>
</task>

Order: Task 1 then Task 2 (real data dependency: Task 1's cross-check and its red test need the old constants in the tree).
Integration step (main session, after an independent review GO): PROGRESS.md entry, LEARNING_PATH §6/§9/§10 (why a teacher needs
margin when a student copies it; what p5 means), COMMANDS.md block "Teacher clearance", commit.

## 7. Tests that must stay green (g1-ext env unless noted)

- `tests/test_integration_volleyball_grasp.py` (all 7; `test_nominal_trial_passes`, `test_trial_passes_with_the_robot_shifted_and_turned`
  run the teacher).
- `tests/test_integration_vla_eval.py`: `test_no_walk_handovers_match_69s_sequence`, `test_replay_through_the_wire_passes_the_gate`,
  `test_teacher_on_the_recorded_handover` (records a fresh demo with 69's `record()` into tmp_path, so it uses the new teacher - it is
  the de-facto 69 smoke), `test_stale_observation_fails_the_gate`. `test_no_walk_seed_101_reproduces_the_recorded_handover` reads the
  T7 and compares only the handover state (before the teacher): unaffected, but it errors while the T7 is unmounted - compare by name.
- `tests/test_integration_volleyball_walk.py`, `tests/test_integration_volleyball_scene.py`, `tests/test_unit_labview.py`.
- Everything in `scripts/run_tests.sh` (project venv, g1-ext, openpi): no new failure by name (G10). Known red before the change
  (audit 10-27): `tests/test_unit_runbook.py::test_runbook_clearance_table_matches_report` - unrelated, stays as it is.

## 8. Do the "ball not lifted" failures share the cause? (analysis only)

Data: 40k 7 lift failures, 45k 3 (2 starts overlap), eval rows + the teacher probe on the same seed-0 starts
(`/tmp/l1day1/lift_analysis.txt`).
- **Teacher margins are not different on those starts:** pinky-table 2.07 vs 2.14 cm (passes), pinky-stand 2.46 vs 2.51 cm, teacher
  lift 9.2 vs 9.2 cm, touch error 0.39 vs 0.44 cm, ball slip 5.6 vs 5.6 mm. The clearance cause does not explain them.
- **They cluster at the far end of the arrival range:** stance `back_m` >= 3 cm in 6/7 (40k) and 3/3 (45k) lift failures vs 36-37 % of
  the passes (range 0-6 cm); mean 3.8 vs 2.6 cm, permutation p = 0.12 (40k), 0.46 (45k) - suggestive, not significant at n = 7.
  5/7 and 2/3 are `no_progress` (the hands hover at the ball until the 45 s timeout, nothing is dropped).
- Contact failures show a waist-yaw lean at 40k (mean -3.6 vs -0.1 deg, p = 0.017) but not at 45k (p = 0.40): inconclusive.
- Reading: "not lifted" looks like the generalization gap at the edge of the stance range (the 10-16 decision rule already said
  generalization gap). Shared with the contacts only in the wide sense that both are a student off the teacher's path near the ball;
  the clearance fix makes such deviations harmless for contacts, it does not make the student close the grasp. Expect the contact
  share to drop with run2 and the lift share to stay unless the data changes.
- Cheapest discriminating follow-up (not in this plan; inference only, ~5 min, no training): re-run the 7 lift-failure starts with
  the 40k server and log per tick the palm-to-ball distance and which hand parts touch the ball at the squeeze time. If the hands
  never reach the touch pose -> oversample back 3-6 cm in v2 (or narrow `arrival_back_m`); if they reach it but do not squeeze ->
  a timing/latch issue of the student.

## 9. Second-order consequences and risks (review lenses walked: `~/Documents/ClaudesOrdner/_harness/templates/project/.planning/REVIEW-LENSES.md`)

1. Guard coverage: G7 measures elbow-down parts only. Upper arm/shoulder were not measured; none of the 600 teacher runs nor any of
   the 18 student contact failures logged an upper-arm contact (all were hand parts). Legs are excluded by design (the knee's 4.1 cm is fixed
   by the stop radius). Distractors (fixed scene) are in FORBIDDEN and measured when `--clearance` is used there; parked ones are 10 m away.
2. What becomes reachable: a shallower cradle (10 deg) is closer to the equator wedge that held 8/36 in the 10-07 probe. Measured
   hold: 789 of 790 starts pass (600 walk incl. physics randomization mass 0.8-1.2, friction 0.8-1.2, + 190 fixed; the one failure is
   s104_t0072, which fails identically with the old constants), lift min 8.2 cm, slip
   p95 7.5 mm (was 6.2). The release now opens more sideways (palm normal more horizontal); basket clearance rose, not fell
   (R wrist-basket p5 1.1 -> 5.0 cm). Fixed rand0 #37 is the edge case to watch: it fails at CRADLE 10-15 without the 3 cm squeeze.
3. Whose meaning changes: every new demo. The 40k model, `tests/data/replay_demo.npz` (robot day) and 87's clearance report stay on
   the old teacher; they are not re-made here. **Mixing v1 (or the partial v2add seeds 105-108, old teacher) with new demos would teach
   two grasps** whose average is the near-miss path again -> ruling R2.
4. Primitive: `mj_geomDistance` with distmax returns distmax when farther and the signed distance (< 0 = penetration) when closer;
   MuJoCo 3.13 in g1-ext (probe used it). Box-mesh pairs work (probe values match the contact log: the one contact start shows -0.1 cm).
5. Shared constants: `CRADLE_DEG`, `REACH_ABOVE`, `SQUEEZE` are read only by `ball_grasp.py` (grep 2026-10-27). 67, 69, 82 `--teacher`,
   the walk/vla_eval integration tests and the lab view use the teacher, so all of them change behaviour - they are in the gate.
Other: the student benefit only shows after v2 demos (needs a disk) + run2 training (GPU or a Mac night) + held-out eval; until then
this is a teacher-level result. Expected (estimate): contact failures 6-12/50 -> ~1-3/50; lift failures unchanged.

## 10. Rulings open for Finnegan (recommendation in brackets)

- R1: accept the visible grasp change (palms 10 instead of 20 deg below the equator, reach end 2 cm higher, squeeze 0.5 cm firmer)
  [yes - it is the only measured variant reaching 3.5 cm without losing a start; fallback CRADLE 15 + REACH_ABOVE 0.06 = 2.9 cm, keeps
  the squeeze].
- R2: v2 dataset recorded entirely with the new teacher; no v1 or v2add demos mixed in [yes].
- R3: gate X = clearance p5 >= 3.5 cm, min >= 3.0 cm, as a working number from 2-7 contact events [yes; re-derive after run2].
- R4: replace the day-plan check "0 finger contacts on the 200 sweep" (already true today) with G7 [yes].
- R5: run the 5-minute lift-failure diagnostic from section 8 before recording v2 [yes, after this change; it decides whether v2
  oversamples far stances].

## 11. Effort

Task 1 ~1-1.5 h, Task 2 ~0.5 h + ~10 min sim (parallel runs, ~1.7 s per walk trial with the meter) + ~3 min tests. One review round.
