# Gate D2: bottle tilt, lens MEASUREMENT VALIDITY

Reviewer: independent gate (Opus), not the author. No access to the author's reasoning. Status as of 2026-10-02, about 14:25.
Checked: `git diff` of `g1sim/grasp.py` and `docs/LEARNING_PATH.md`, the new file `scripts/63_grasp_sweep.py`, the evidence in `runs/`, the
exploration outputs in the scratchpad folder `d2/`, all consumers of the changed constants.
`scripts/64_demo_factory.py` (also untracked, D4) does not belong to D2 and was not assessed.

## What I computed or ran myself

| Check | Result |
|---|---|
| `54 --randomize --trials 50 --seed 0`, current code, output to the scratchpad | **bit-identical** to `runs/d2_54_seed0.json` (all 50 `results` identical, top-level fields identical) |
| The same with the constants before D2 (HEAD: `GRASP_ROTATION` without roll, `CLOSED=ones(6)`, no `LOAD_SAG`), seed 0 and seed 1 | Seed 0: 19/50, reachable 20, 19/20, tilt median 41.74 / max 53.03. Seed 1: 27/50, reachable 31, 27/31 = 87.1 %, median 39.05 / max 54.19 |
| `58_teleop_check.py` with current code (outputs redirected to the scratchpad, code via symlink, not copied) | **ABNAHME BESTANDEN 16/16** (acceptance passed), episode lift 10.3 / 9.3 / 8.8 cm, same as the author's |
| Steps of `tests/test_integration_grasp_ik.py` rebuilt as a separate script (no pytest) | see F1: **the 23-DoF case fails** |
| Force limits of the right hand read from the model | thumb rotation motor (thumb_metacarpal) 0.5 Nm, ThumbAux 1.1 Nm, each finger 2 Nm: matches the reasoning in grasp.py |

Key figures recomputed from the JSON files (tilt and lift only over passed trials):

| Run | passed | reachable | Success with reachable bottle | Tilt median / max | Min lift | Hand error max |
|---|---|---|---|---|---|---|
| before `l3a_54_alles` (seed 0) | 19/50 | 20 | 19/20 = 95 % | 41.29° / 54.25° | 7.42 cm | 1.31 mm |
| before, my re-run on current code (seed 0) | 19/50 | 20 | 95 % | 41.74° / 53.03° | 7.33 cm | 1.30 mm |
| before `d2_basis` (63, seed 0) | 19/50 | 20 | 95 % | 39.91° / 50.74° | 7.43 cm | n/a |
| **after `d2_54_seed0`** | **25/50** | 25 | **25/25 = 100 %** | **2.71° / 4.37°** | **7.75 cm** | 5.96 mm |
| before, my re-run (seed 1) | 27/50 | 31 | 87.1 % | 39.05° / 54.19° | 5.31 cm | 8.88 mm |
| **after `d2_54_seed1`** | **35/50** | 35 | **35/35 = 100 %** | **2.40° / 3.93°** | **7.43 cm** | 1.90 mm |
| fixed start setup after `d2_nachher` | 5/5 | 5 | 100 % | 2.42° | 9.97 cm | 1.02 mm |

Fairness of the comparison:
- **Same start setups:** `bottle_start`, `variation` and `start_draws` are the same in all 50 trials (seed 0: l3a vs. d2; seed 1: my
  "before" run vs. d2). Start setups come from `env.rng`, the image noise from a separate random stream (`table_env.py:42-44`). So the way the
  grasp goes does not shift the start setups.
- **Same definition of success:** `scripts/54_grasp_scripted.py` was last changed at 11:36, so before D2. 63 imports `evaluate` from 54
  (`63_grasp_sweep.py:43-46`) and does not rebuild it.
- **Start-by-start comparison:** In all 19 (seed 0) and 27 (seed 1) start setups that pass both before and after, the tilt drops from
  30–54° to 0.1–4.4°. Median on these shared starts: 41.3° → 2.2° (seed 0) and 39.1° → 2.4° (seed 1).
- **Overfitting to seed 0:** The search ran on seed 0 with about 60 variants (scratchpad `kombi/r1..r4`). The candidates then ran on
  seed 1 (`kombi/s1`), and the final version ran through 54 on seed 1: 35/35, 2.4° / 3.9°. Against my "before" run on seed 1 (27/31, 39°) the
  gain is just as large as on seed 0. I find no sign of overfitting.
- **No change to physics or contacts:** `git status` shows only `grasp.py`, `LEARNING_PATH.md` and two untracked scripts.
  `configs/randomization.json` is from 16.09., `g1sim/table_scene.py` from 22.09. In `~/dev/g1-ext/g1_brainco_sim`, after `BUILD.json`
  (10:39) only `assets/scene_29dof_brainco_table.xml` is newer, and it is rewritten on every run from the unchanged `table_scene.py`.
  Leaving the ring finger and pinky open and rolling the hand is a change to the control, not to the model.

D2 acceptance check against the criteria:
- Tilt median at least 10° lower than before, target ≤ 25°: **2.71°** (−38.6°). Met by a wide margin.
- passed ≥ 19/50: **25/50**. Met.
- Success with reachable bottle ≥ 95 %: **25/25**. Also met with the same denominator (see F3).
- fixed start setup 5/5: **5/5**. Met (see F5).
- 58 still passes: **16/16**, re-run by me.
- no change to physics or contacts: met.

## Findings

### F1 (major): With the D2 grasp posture, a 23-DoF G1 no longer reaches the grasp pose. An existing integration test fails, and this is not disclosed
- **What:** `GRASP_ROTATION` is now rolled by −30° around the finger axis (`g1sim/grasp.py:30-31`). This constant is also read by
  `tests/test_integration_grasp_ik.py::test_ik_erreicht_greifpose[23dof]` (lines 26-42). The test checks `pos_err < 0.01` when the
  wrist is locked (pitch/yaw locked, so 23 DoF).
- **Evidence** (steps of the test rebuilt in a fresh process, `TableEnv()`, `reset()`, `ArmIK(lock_wrist=True)`, `solve(grasp_pose(...))`):
  - D2 `grasp.py`, 23 DoF: **pos_err 40.67 mm**, rot_err 0.46°. The test fails (limit 10 mm).
  - Constants before D2, 23 DoF: 7.37 mm, the test passes. 29 DoF: before 1.31 mm, after 0.79 mm, both pass.
  - Sweep over the roll angle at 23 DoF: 0° 7.4 mm · −5° 13.3 · −10° 19.2 · −15° 24.9 · −20° 30.5 · −25° 35.7 · −30° 40.7 mm.
    Even −5° breaks the limit. The whole plateau −15 to −35°, which grasp.py gives as the reason, cannot be reached with 23 DoF.
  - `test_ik_haelt_abstand_zum_rumpf` still passes (distance arm–trunk 3.94 mm).
  - I did not run pytest (rule of the task). The test run itself is still open for the user.
- **Consequence:** (1) When the user runs the tests, the suite turns red. This is not written anywhere in the handover. (2) Whether the G1 has 23 or 29 DoF
  is open (memory "G1 DOF blocker", PROGRESS:49), and exactly this question blocks D5. D2 makes the 23-DoF path measurably worse: the
  static IK error rises to 5.5 times. LEARNING_PATH §10.10/§10.11, however, says: "Not solved by D2: … 23/29 DoF" and "it does not block D2
  and D4". That D2 makes the 23-DoF case worse is not written there.
- **Proposal:** Disclose it to the user and let them decide. Either (a) apply the roll only at 29 DoF, so `grasp_pose`
  gets the rotation depending on `lock_wrist`, or (b) mark the 23-DoF case in the test as `xfail` with a reason (D2 + DoF blocker),
  or check the unrolled `SIDE_GRASP_ROTATION`. After that, correct the entry in LEARNING_PATH §10.10/§10.11 and PROGRESS (D3 rest).

### F2 (note): The "before" file cannot be recomputed bit-identically with today's code. The conclusion stays the same
- **Evidence:** `runs/l3a_54_alles.json` was created at 11:37:57. `g1sim/table_env.py` and `g1sim/episode.py` were changed after that (11:42, D3:
  `observation()` now renders only `head_image`). Because of this, the noise stream uses different numbers, and my "before" re-run differs slightly
  trial by trial. Example trial 3: tilt 48.1° vs. 52.1°. The totals are the same: 19/50, 19/20, median 41.7° instead of 41.3°, max 53.0° instead of 54.3°.
- **Consequence:** None for the verdict. The gap to 2.7° is huge. But as a reference for later comparisons, the file is not reproducible.
- **Proposal:** Store a "before" run on the current code (e.g. `runs/d2_vorher_54_seed0.json`, `..._seed1.json`) and name this source
  for the numbers in the log.

### F3 (note): What counts as "reachable" depends on the grasp pose; the denominator grew from 20 to 25. With the same denominator the criterion still holds
- **Evidence:** `reach_error()` (`grasp.py:94-107`) measures against `grasp_pose()` and so against the new `GRASP_ROTATION`. Seed 0: newly
  reachable are trials 10, 16, 38, 42 and 44. Their reach error fell from 12.7–23.2 mm to 3.8–8.6 mm. Seed 1: new are 0, 7, 13 and 40
  (21–24 mm → 6.5–8.6 mm). On the old reachable set, D2 passes **20/20** (seed 0, before 19/20) and **31/31** (seed 1, before 27/31).
  No start that was reachable before is unreachable now.
- **Consequence:** With 95 %, the compared set is not exactly the same. But on the shared set the criterion is also met, and
  more reachable starts is a real gain (passed 19 → 25).
- **Proposal:** Give both numbers in the report: "25/25 with the new denominator, 20/20 on the old 20 reachable starts".

### F4 (minor): For the newly reachable edge starts, the margin to some limits is small
- **Evidence:** Reach error up to 8.63 mm (seed 0, trial 10) and 8.55 mm (seed 1, trial 13), limit 10 mm. Hand error before closing
  **5.96 mm** (seed 0, trial 42, limit 10 mm; all others ≤ 1.9 mm). Min lift 7.75 cm and 7.43 cm (limit 5 cm; the numbers in the
  grasp.py comment, 7.7 and 7.4 cm, are right). The four starts with the smallest lift are exactly the edge starts with reach error 4–9 mm.
- **Consequence:** Two seeds with 60 reachable starts in total have no failure. A third seed could contain starts with 9–10 mm
  reach error, where `hand_an_greifpose` fails. So "100 %" is not a promise of robustness.
- **Proposal:** Before the mass recording (D5), run a third seed (e.g. `--seed 2`) and also report the hand error over all starts.

### F5 (note): The fixed start setup "5/5" is a single trial, repeated five times deterministically
- **Evidence:** In `runs/d2_nachher.json` all 5 trials are identical: tilt 2.422500839°, lift 0.099677748 m, hand error 1.02307 mm.
  `runs/d2_vorher.json` even has only `trials=1`.
- **Consequence:** It is fine as a regression check. Statistically it shows only one case. This has been so since Stage 1.4 and is not specific to D2.
- **Proposal:** None needed. In the report, write "deterministic, 1 case" instead of "5/5" as a rate.

### F6 (minor): The evidence for the chosen constants is only in the temporary scratchpad. The variant files refer to the old base posture
- **Evidence:** The plateau −15 to −35°, "from −40° no longer 5 cm" (`d2/C/*.log`: rz−40 with closed ring finger, min lift 4.9 cm), the
  thumb 1.0 → 0.70 (`d2/D/trace_*.txt`) and the seed 1 cross-check (`d2/kombi/s1.log`) are only under
  `/private/tmp/claude-501/.../scratchpad/d2`. grasp.py points to "log 2026-10-02", but `PROGRESS.md` is unchanged and has no
  D2 entry. Also: `63_grasp_sweep.py` turns `ROT_DEG` on top of the current `grasp.GRASP_ROTATION` (`BASE`, line 50/65). Since D2 this is
  already rolled, and the same variant file (`"ROT_DEG": [0,0,-30]`) would give −60° in total today.
- **Consequence:** After a restart or logout, the numbers in the comments can no longer be backed by evidence. Anyone who repeats the sweep
  rolls twice without noticing.
- **Proposal:** Copy the most important sweep outputs to `runs/d2_sweep_*.json` (stays local, `runs/` is gitignored), write a
  PROGRESS entry with commands, and record there that the variant files refer to the base posture before D2.

### F7 (note): Small inaccuracies in the text
- LEARNING_PATH §10.10: "grasp lower (7.5–12 cm) … (44–50°)". Measured are medians of 42–50° (`d2/A/coarse.log`: h105 = 42°).
- grasp.py:28: "−15 .. −35 degrees, all … 2–4 degrees tilt". This holds for the medians (4/3/2/2/3); the maxima go up to 5°.
- **Proposal:** Align the wording. This changes nothing in the results.

### F8 (note, on the user's question "wrist camera image, we do not have that"): The user is right. The image comes from a sim-only camera, and no D2 measurement depends on it
- **Evidence:** `54_grasp_scripted.py:44-47,113` renders `right_wrist_camera` and writes `docs/<name>_handkamera.png` (so also
  `docs/d2_54_seed0_handkamera.png`, `d2_54_seed1_…`, `d2_nachher_…`) and `wrist_camera_red_fraction` into the JSON. The real G1 has no
  wrist camera (D3, commit b48cf34). Measuring uses only the head camera: `Locator` uses `head_camera` by default
  (`perception.py:23`), and the dataset holds only `head_image` (`episode.py:20`). LEARNING_PATH §10.9 keeps the sim wrist camera on purpose "display
  only".
- **Consequence:** None for the D2 numbers. But the images `*_handkamera.png` and the log line "Handgelenkkamera, Anteil rote Pixel" (wrist camera, share of red pixels) give the
  impression that the robot has such a camera.
- **Proposal:** Let the user decide: switch the check images in 54 to the head camera or an outside view, or label them clearly as "sim
  only, display only". This is not part of the D2 scope.

### F9 (note): The author's teleop evidence is borderline in timing, so I re-ran it
- **Evidence:** The last change to `grasp.py` was at 14:10:22. The author's run of 58 ended at 14:11:32; with about 71 s run time (per
  `d2_teleop_check.time`) it started at about 14:10:21. Whether this run imported the final version is open. My re-run on the
  final version passes 16/16.
- **Consequence:** None. My re-run backs the criterion.

### Side effect of the review (for transparency)
My first run of 54 with the working directory in the repo appended a MuJoCo warning to the gitignored `MUJOCO_LOG.TXT`
(14:13:39). Apart from that, I did not create or change any file in the repo: all outputs of 54 and 58 went to the scratchpad, the author's
evidence in `runs/` is unchanged (mtime of `runs/teleop_check.json` 14:11:32). I briefly created a code copy for 58 and deleted it again
unused, because the rule is: code only in the repo. 58 ran via symlinks instead.

## Verdicts

**(1) Did the code do what the spec asked?** Yes. All agreed acceptance criteria are met and recomputed by me:
2.71° median (seed 0, reproduced bit-identically), 25/50, 25/25 or 20/20 on the old set, fixed start setup passed, 58 16/16, no
change to physics or contacts. Seed 1 confirms the result: 35/35, 2.4°.

**(2) Was the plan right?** Mostly yes. Measured instead of guessed, same start setups, same definition of success, cross-check on a
second seed, contact model left alone on purpose. It has one gap: the checklist covered only 54 and 58 as consumers of
`GRASP_ROTATION`, not the IK integration test for 23 DoF and not the open 23/29-DoF question. The roll buys less tilt at the cost of
reachability with a locked wrist, and this is not disclosed (F1).

**Overall: NO-GO**, only because of F1. The measurement itself is valid (GO for the measurement lens). The verdict turns to GO as soon as F1 is either
fixed (roll depending on 29/23 DoF) or the user has explicitly accepted it and the test and the documentation are updated.
F4 and F6 should be done before D5; F2, F3, F5, F7, F8 and F9 are notes.
