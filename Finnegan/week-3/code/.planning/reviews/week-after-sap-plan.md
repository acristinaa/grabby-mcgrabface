# Plan review round 1 - `.planning/after-sap/WEEK-2026-11-16-VISIONPRO-CORRECTIONS.md`

Reviewer: independent (did not write the plan). Date 2026-11-09. Only files read, no code run (Mac budget).
Checked against: g1sim/table_env.py, vla_eval.py, ball_grasp.py, teleop.py, episode.py, policy_wire.py, real_safety.py,
grasp_ik.py; scripts/57, 69, 79, 80, 82, train_smolvla.sh; PROGRESS.md; lerobot 0.6.1 in ~/dev/g1-ext/smolvla-test/.venv.

## Verdicts

- **Is the plan right?** Mostly yes. Teacher-first, then human, all in sim, teacher demos kept in the mix, paired eval on
  seed 0, strict GPU order: all sound. But four points are wrong about our own code or data (F1-F4).
- **Is it doable as written?** No. As written, the Wed check "80 passes on the merged set" cannot pass (F1, F2), the
  weighting option B cannot be built the way it is described (F3), and the snapshot ring misses the moments needed for one
  failure class (F4). The Wed schedule is also too full (F5).
- **Overall: NO-GO** for this round. All findings can be fixed in the plan text plus a small, planned change to 80.

## Major

**F1 - Validator 80 rejects every correction episode.** `scripts/80_validate_dataset.py:50-51` fails any sidecar with
`seed < 100`, but the plan harvests on seeds 2-4 (82 refuses >= 100 with `--no-walkup`, `scripts/82_vla_eval.py:121-122`).
`80:53-56` fails any two episodes with the same `variation`: 1-3 snapshots from one failed trial share it, and so do the
copied episodes of Option A and the x2 in Option B. 80 also needs `handover_json`, `stance_source` and `timestep` (`80:42,
52-55`), which teleop's sidecar does not write (`g1sim/teleop.py:298-301`). The plan marks "80 with duplicated episodes" as
unverified. It is verified now: 80 rejects them. Fix: plan a small 80 change (a `source: correction` sidecar may have seed
2-99 and share its trial's variation; copies are allowed when tagged as copies) and list the required sidecar fields in 3.2.

**F2 - 80 cannot check a merged dataset.** 80 reads `<root>.convert.json`, written only by 79 (`80:131-132`, `79:111-113`).
A dataset built by `lerobot-edit-dataset --operation.type merge` has no such map, so the Wed-evening check can't run.
Options: run 79 once over the teacher folders plus the correction folders (79 takes several folders, `79:52`). That
re-converts 1,691 demos, so it takes hours, and the lossless images are ~30 GB per 300 episodes (PROGRESS.md:1154), so
roughly 170 GB. The other option is to make 80 write or read a map for merged sets. In both cases the PC's free disk after
v2 is not known (849 GB before v2, PROGRESS.md:119). The merge CLI flags are marked (R) and I did not check them.

**F3 - "Corrections x2" cannot be done with `--dataset.episodes`.** The sampler turns the list into a yes/no mask
(`lerobot/datasets/sampler.py:81-84`, read): an episode listed twice still counts once. So x2 needs physical copies, which
runs into the same problem as Option A (F1). The plan calls B "lighter" for the wrong reason. A simpler fix with no copies:
take fewer teacher demos. Corrections x1 = 90k frames; for 33 % you need ~180k teacher frames (~250 demos at 726 frames).
With 500 teacher demos and x1 they are only ~20 %. Redo the 3.3 and 4.1 numbers.

**F4 - A ring of the last 4 s misses the "failed the lift check" class and weakens "never lifted".** Those 10/100 trials
reached placing with the ball in the basket, then timed out at 45 s (`vla_eval.py` docstring: the timeout reason is the
first failed check). The last 4 s show a ball that has lain in the basket for a long time, and nobody can correct from
there. Yet Wed pm puts "ball in basket without lift" first for the humans. For "never lifted" (stuck after the handover,
PROGRESS.md:48) the last 4 s are just the stuck pose. Fix: also save snapshots at events (handover, first ball contact,
entry to `placing`), or keep a sparse ring over the whole trial (one snapshot every 0.5 s for 45 s is ~90 small arrays).
Then choose the takeover point per class.

## Medium

**F5 - The Wed critical path is too full for 3 people.** Tue builds the two-arm pilot from nothing. Wed adds fingers,
waist, the headset view and loading a snapshot, then a human session, a data freeze at 18:00 and run4 that evening. The
F1/F2 fixes are not scheduled at all. Mon for Finnegan holds the 1 h decision, the snapshot ring (0.5-1 d), deploying it
to the PC and the harvest, and Tue's sorter gate depends on that harvest. Suggestion: make D2's default "run4 is
teacher-only" and send human corrections to round 2. Schedule the 80/79 fix on Tue (Finnegan).

**F6 - Correction length and ending don't match the teacher, so the frame math is off.** Teacher demos run from the
handover to the end of `face_front` + 30 ticks and are 600-800 frames; settle and check are not recorded
(`scripts/69_volleyball_demo_factory.py:8-11`). A teacher takeover from a "never lifted" state replays almost the whole
sequence (face, reach, touch, squeeze, lift, turn, centre, release, retract, face_front; `ball_grasp.py:204-252`), which is
~500-700 frames, not ~300. Human episodes must also end with retract + waist back to 0, like the teacher, or the policy
learns to stop at the basket. Re-size 3.1/3.3/4.1 with the measured lengths and add "ends like a teacher demo" to 3.4.

**F7 - New normalization stats on a fine-tune.** With `--policy.path` and no `resume`, lerobot 0.6.1 replaces the
checkpoint's stats with `dataset.meta.stats` of the new dataset (`lerobot_train.py:350-351, 375-377`, read; the plan says
the same). That shifts every normalized input and output of the run3 weights. The stats will most likely come from the
whole merged set, not the subset (that is the dataset's stored meta; I did not run it). The demo envelope is built from
`stats.json` (`g1sim/real_safety.py:47`), so it moves too. Add a check: compare the old and new stats per channel before
starting run4, and re-derive or freeze the envelope.

## Minor

- **m1** 4.1 uses 1.7 s/step, but run3 measured ~1.84 (PROGRESS.md:55, `train_smolvla.sh:38`): 30k steps = 15.3 h + ~35 min
  loading. Start 20:00 means done Thu ~12:00, not 11:00. Eval Thu pm still fits.
- **m2** The Mon takeover check (>= 45/50 from the teacher's own mid-grasp snapshots) needs teacher runs to make the
  snapshots plus 50 takeovers, ~100 runs. The Mac rule is <= 50. Do 25 + 25, or run it on the PC CPU.
- **m3** The snapshot ring runs `mj_forward` inside the eval loop (`table_env.py:262`). Add a check: 5 seed-0 trials with
  the ring on and off give identical rows. Otherwise the run4-vs-run3 paired table is not like for like. Or keep the ring
  off for seed-0 evals.
- **m4** Say the scene by name: the snapshots come from `volleyball_walk` (`vla_eval.py:24`). 57 builds a default
  `TableEnv` (bottle, `57_teleop.py:203`). A restore needs the same scene.
- **m5** Tue-eve bulk teacher on the PC CPU: a 69 process leaks ~1.8 -> 3-4 GB over ~40 demos on the PC (PROGRESS.md:123-124).
  Reuse the 40-per-process batches and `runs/pc_watchdog.sh`.
- **m6** Decision table: the rows `<= 55` and `55-84` overlap at 55. Missing SAP branches: visit cancelled or robot not
  run, and a safety stop or hit on the real robot that is not the O1 case.
- **m7** "Pick the best run3 checkpoint, not always the last" (D4) means several 100-trial evals on the GPU. Nobody has
  sized that time.
- **m8** Wrist envelope: 18/100 teacher trials go above `ENVELOPE_HI` (fence-smooth.md:114). The 3.4 filter will therefore
  also drop some teacher takeovers. Expect that loss in the counts.
- **m9** Jargon with no plain-words line: IK, FK, Wilson interval, HG-DAgger/RaC/Sirius/IWR (one line each), mj_forward.

## Checked and correct

Snapshot handover saves and restores the variation, controller targets, pending latency queue and camera-noise RNG
(`table_env.py:258-289`). Eval trial loop `vla_eval.py:361-393`. Seed rules `82:10-11, 121-122`. `FORBIDDEN`
`ball_grasp.py:102`, `placement_checks` :138, two `ArmIK` :182, `ready_pose` start :196, holding fingers :88. `TASKS`
`policy_wire.py:37`, teleop uses openpi's `DEFAULT_TASK` (`teleop.py:22`), right arm only (`teleop.py:25`), PhonePilot
clutch (`teleop.py:190-214`). 57 binds 127.0.0.1 (`57_teleop.py:209`). MultiLeRobotDataset off (`factory.py:124`).
Weighting only `rabc`/`uniform` (`sample_weighting.py:125-132`). Envelope `real_safety.py:48-52`. IK guards
`grasp_ik.py:76-80`. Epoch math: run2 0.59 + run3 0.98 = 1.57; Option B 0.44 epochs and Option A 0.14 are arithmetically
right for their stated inputs (inputs wrong, see F3/F6). Failure mix PROGRESS.md:48, load time :71. Web: avp_stream latest
is 2.51 and 2.50 exists (https://pypi.org/pypi/avp_stream/json). Tracking Streamer is free and needs visionOS 2.1+
(https://apps.apple.com/us/app/tracking-streamer/id6478969032).

## Not verified

avp_stream's scene upload over HTTP (`streamer.py:3756-3765`; the package is not installed here). `lerobot-edit-dataset`
merge flags. Whether subset training stats come from the whole set (not run). Free disk and time per eval trial on the PC.
Vision Pro rate and latency.

# Plan review round 2 - revision 1 of `.planning/after-sap/WEEK-2026-11-16-VISIONPRO-CORRECTIONS.md`

Reviewer: independent (did not write the plan or round 1). Date 2026-11-09. Only files read, web checked; nothing run.
Re-read: g1sim/vla_eval.py, table_env.py, ball_grasp.py, teleop.py, episode.py, policy_wire.py, real_safety.py; scripts/57, 69,
79, 80, 81, 82, train_smolvla.sh; PROGRESS.md; lerobot 0.6.1 (`scripts/lerobot_edit_dataset.py`, `datasets/dataset_tools.py`,
`datasets/aggregate.py`, `datasets/io_utils.py`, `datasets/sampler.py`, `scripts/lerobot_train.py`) and `datasets/io/parquet.py`
in ~/dev/g1-ext/smolvla-test/.venv.

## Verdicts

- **Is the plan right?** Mostly. Teacher first, run4 teacher-only, snapshots + per-class takeover, stats freeze, paired eval:
  sound. Two parts are not right: the merge route for Option B (N1) and a Monday check that cannot prove what it claims (N2).
  The run4 size also breaks the project's own epoch rule without saying so (N3).
- **Is it doable as written?** No. The Wed-am merge most likely does not fit the PC disk and is not timed, so the 14:00/20:00
  run4 start is at risk (N1). The Mon on/off check needs a VLA (not allowed on the Mac) and compares fields that always differ (N2).
- **Overall: NO-GO.** All fixes are plan text; N1's fix makes the plan smaller (drops the "80 reads a list of maps" change).

## Round-1 findings, re-derived

| # | Status | Basis |
|---|---|---|
| F1 | fixed | 80 change on Tue; seed/variation rules and missing sidecar fields match `80:50-56` and `teleop.py:298-301` |
| F2 | partly | list-of-maps plan is consistent; merge order is now verified (`aggregate.py:426-441` loops the sources in order, `dataset_tools.py:271-303`). But the disk check is wrong, see N1 |
| F3 | fixed | mask sampler confirmed (`sampler.py:81-84`); Option B numbers re-computed: 200 x 600 = 120k, 2x = 240k = 331 demos, 33 %; 500 demos -> 24.8 %; Option A K = 0.4286 x 1,227,666 / 120k = 4.4 |
| F4 | fixed | event snapshots + 0.5 s ring; per-class takeover points. Small slip: "first robot-ball contact" is not a tracker event (`vla_eval.py:39` has handover/lifted/carried/placing/latched; `measure()` :168-173 reports only forbidden contacts), so it needs its own contact check |
| F5 | fixed | run4 teacher-only, 80 work on Tue |
| F6 | fixed | ~600 frames: the teacher timeline from face_ball to face_front + 30 ticks is ~23-24 s (`ball_grasp.py:90-92, 204-252`) |
| F7 | fixed, with a new side effect | freeze is the right lever (`lerobot_train.py:350-351, 375-377` take `dataset.meta.stats`); see m-b |
| m1-m9 | fixed | m3 only on paper: see N2. 4.1 math checked: 30k x 1.84 s = 15.3 h; 14:00 + 15.9 h = Thu ~06:00; 240k / 360k = 0.67 |

## Major

**N1 - The merge route does not fit the PC disk and is not timed; the simpler route was rejected for a wrong reason.**
- What the merge really writes (lerobot 0.6.1, read): (1) a full copy of v2 + corrections (~195 GB); (2) `aggregate_data`
  reads every source file with `datasets.Dataset.from_parquet` (`aggregate.py:620-621`), which builds an Arrow cache on disk
  (`datasets/io/parquet.py` `read` -> `download_and_prepare`), most likely a new one, not the training cache (no `features`
  passed, so a different cache key; not verified); (3) `merge_datasets` then loads the whole merged set (`dataset_tools.py:305`),
  building its full Arrow cache (~30 GB per 300 episodes, PROGRESS.md:1157). That is ~400-590 GB, plus hours of IO on a disk
  that already showed 76 % iowait (PROGRESS.md:71).
- Free disk now (inferred, not measured): 849 GB before v2 (PROGRESS.md:119) minus v2 raw npz ~198 GB (35.1 GB per 300 demos,
  PROGRESS.md:1153), v2 dataset ~170 GB, v2 Arrow cache ~170 GB = ~310 GB. The plan's "< 250 GB free -> move files" threshold is
  too low, and its fix "move to the T7" may not work: the T7 is not seen by the PC (PROGRESS.md:122; not re-checked since).
- Then 80 on the merged set re-reads all 1,691 + N npz files several times (`80:37, 81, 109`): also hours, not timed.
- Option B needs only ~330 teacher demos. Converting **those plus the corrections with 79** (a folder of symlinks to the chosen
  npz files, `79:47` globs `episode_*.npz`) gives one ~360k-frame dataset (~50 GB + ~50 GB cache), in roughly 10-20 min
  (79 did 300 demos in 7 min, PROGRESS.md:1154), with one `convert.json`, so 80 works with only the seed/variation change.
  Training needs no `--dataset.episodes` then. The plan rejected this because it "re-converts all 1,691 demos (hours,
  ~170 GB)"; Option B does not need all 1,691. Fix: make this the Option B route, drop the list-of-maps 80 change and the
  merge, keep `df -h` with a "needs ~120 GB" bar. Option A keeps the merge problem; say so in D1.

**N2 - The Monday on/off check (m3) cannot prove "capture does not change the eval" as written.**
- It needs a policy. The row says "Mac (code), tiny test", but a VLA on the Mac is forbidden, and the PC GPU is the only place
  for one. On CUDA the same seed is promised identical only on CPU (`81_smolvla_server.py:7-8`), so GPU runs may differ for
  reasons that have nothing to do with capture.
- "Identical per-trial rows" can never pass: every row has `wall_s` (`vla_eval.py:275`) and the server/round-trip times.
- Fix: run on/off on the Mac with `82 --teacher` (no model, deterministic) or `81 --replay` (no model), 5 trials, and compare
  passed, reason, event ticks and the executed actions bit for bit, not wall-clock fields.

## Medium

**N3 - run4 at 0.67 passes over the corrections breaks the epoch rule that came out of run2.** PROGRESS.md:51-53: "size
training in epochs (target 1.5-2)". run2's 0.59 epochs is exactly why it scored 47. The plan argues the teacher frames were
already seen 1.57 times, but the corrections are new states, seen 0.67 times. 1.5 passes at a 33 % share = 540k samples =
67.5k steps = 34.5 h, which does not fit. Options for D4: a 50 % share (IWR) -> 45k steps = 23 h (done Thu ~13:30 if started
14:00, eval Thu night/Fri), or keep 30k and write down that it is a deliberate exception and a weak eval is not proof that
corrections do not help.

**N4 - The decision for "teacher takeover gate fails" is missing.** Tue gate: ">= 50 % on never lifted -> bulk corrections".
If it fails, D8 says "DART as plan B", but nobody says what run4 trains on Wed (only the classes that pass? DART recorded Tue
night on the PC CPU? no run4?). Done item 4 depends on it. Add one row to 1.1.

## Minor

- **m-a** Wed pilot "watch the 2D stream in the headset browser": not verified that Safari stays visible while Tracking
  Streamer tracks the hands. avp_stream has its own synthetic video channel (`configure_video(...)` without a device +
  `register_frame_callback`, WebRTC; https://github.com/Improbable-AI/VisionProTeleop), which shows our rendered frames inside
  the app and is not the 3D-scene upload the plan forbids. Name it as the first or the fallback route.
- **m-b** Freezing stats (write v2's stats into the new set's `meta/stats.json`) makes 80's stats check fail by design
  (`80:96-102` compares `meta.stats` with numpy over the npz frames). Order: 80 first, then freeze, and note it.
- **m-c** 82 writes the scene XML at start and must not run while recorders read it (`82_vla_eval.py:28`). Tue may run a
  reserve harvest (82, GPU) next to sorter/bulk teacher runs (PC CPU). Say: not at the same time, or check the takeover
  script does not read that file.
- **m-d** Snapshots are taken on the PC (x86) and restored on the Mac (arm64) for the sorter and the human pilot. Restore is
  bit-exact only "in a fresh TableEnv of the same scene" (`table_env.py:272-273`), same machine assumed. Fine for a start
  state, but the Mon replay gate must run on one machine, and both must use the same repo commit and randomization config.
- **m-e** SAP branch "real image/pose differs -> update the sim first": then run3's seed-0 number is from the old scene; run3
  must be re-evaluated in the new scene or the run4-vs-run3 pairing is not like for like.
- **m-f** Line slips: 80 reads `convert.json` at `80:128` (plan says 131-132). The run4 case in `train_smolvla.sh` must pass
  `--dataset.repo_id`/`--dataset.root` of the new set (and `--dataset.episodes` only if the merge route stays).
- **m-g** Jargon still without a plain line: Arrow cache, warm-up/decay schedule, clutch is explained, "side file" = sidecar
  (both words used).

## Checked and correct

`snapshot_handover`/`restore_handover` (`table_env.py:258-289`, saves pending latency queue and noise RNG); trial loop
`vla_eval.py:361-393`; `ready_pose` start `ball_grasp.py:196`; two `ArmIK` :182; `FORBIDDEN` :102; `placement_checks` :138;
holding fingers :88; teleop right arm only (`teleop.py:25`), `DEFAULT_TASK` (:22), PhonePilot clutch (:190-214), speed caps in
`limit()` (:140-148), sidecar (:298-301); 57 binds 127.0.0.1 (`57_teleop.py:209`), default `TableEnv` (:203), slow-motion
recording (:9-10); 82 seed rules (:10-11, 121-122) and TASKS check (:236); 79 glob (:47) and map write (:98); envelope
`real_safety.py:47-52` is fixed constants; merge flags `lerobot_edit_dataset.py:89-95, 292-296, 499-505`; subset training
caches only the chosen rows (`io_utils.py:81-82`); stats from stored meta (`factory.py:98-109`); MultiLeRobotDataset off
(:124). Web: avp_stream latest 2.51 (https://pypi.org/pypi/avp_stream/json).

## Not verified

PC free disk and whether the T7 is reachable from the PC now; whether `from_parquet` in the merge reuses the v2 training cache;
merge and 80 wall time on the PC; PC eval time per trial; GPU run-to-run determinism of SmolVLA; whether Safari is visible
during Tracking Streamer hand tracking; snapshot size.

# Plan review round 3 - revision 2 of `.planning/after-sap/WEEK-2026-11-16-VISIONPRO-CORRECTIONS.md`

Reviewer: independent (did not write the plan or rounds 1-2). Date 2026-11-09. Only files read; nothing run (Mac budget).
Re-read: scripts/79, 80, 81, 82, run2_pc.sh, train_smolvla.sh; g1sim/episode.py, table_env.py:250-295, vla_eval.py (docstring,
:100-108, :345, :403, :444-495), policy_wire.py:33-37, teleop.py:23,230,299; PROGRESS.md:40-130, 1112-1160; lerobot 0.6.1
`scripts/lerobot_train.py:340-380`, `datasets/io_utils.py:161-174`, `datasets/utils.py:93`. `du` of the Mac's run1 checkpoints.

## Verdicts

- **Is the plan right?** Yes. Teacher-first, teacher-only run4, one 79 run instead of the merge, model-free on/off check,
  epoch rule on the corrections, stats freeze after 80, strict GPU order: all sound against the code.
- **Is it doable as written?** Yes, after one small text fix (r3-1). As written, the Wed-am symlink folder has no side files, so
  79 skips every episode; it fails loudly (79 exit 1, 80 exit 1 on `skipped`), so no wrong data can reach training.
- **Overall: GO**, with r3-1 applied before Wed. No new review round needed for it.

## Round-2 findings, re-derived

| # | Status | Basis |
|---|---|---|
| N1 | fixed, one gap (r3-1) | merge dropped; 79 globs `episode_*.npz` (`79:47`), writes one map (`79:96-99`), 80 reads it (`80:128`). Disk table re-computed: 120k x 161 KB = 19 GB; 240k / 360k x 138 KB = 33 / 50 GB; totals 85 / 120 GB |
| N2 | fixed | 81 `--replay` has no torch (`81:4-5`); 82 `--from-handover` + replay runs `replay_gate_problems` (`vla_eval.py:464-492`), which needs `replay_state_error == 0.0` and `obs_state_err_max == 0.0`: any physics change from capture would fail it. Note: with a replay server the actions are the recorded ones by design, so the state checks are the part that proves it |
| N3 | fixed | 45k x 8 = 360k = 1.5 x 240k; 45k x 1.84 s = 23.0 h; 67.5k x 1.84 = 34.5 h; 30k x 1.84 = 15.3 h; end times in 4.1 and the Thu row match (+10 min load) |
| N4 | fixed | 1.1 last row: 100 x 600 = 60k, 1.5 x 60k / 0.5 / 8 = 22.5k steps x 1.84 s = 11.5 h |
| m-a..m-g | fixed | m-b order in 4.2 and Wed-am row; m-c Tue reserve row + risk row (`82:28`); m-d section 2; m-e 1.1 row; m-f `80:128` correct; m-g glossary |

## Medium

**r3-1 - The symlink folder needs the side files too.** 79 checks `path.with_suffix(".json")` on the path it globbed (`79:78`)
and stores that same unresolved path in the map (`79:87`); 80 reads the side file the same way (`80:45`). With a folder of
symlinks to the npz only, every episode gets "no sidecar": 79 writes nothing useful and exits 1 (`79:102`), 80 exits 1 on
`skipped` (`80:141`). Fix (3.5 and the Wed-am row): link both `episode_*.npz` and `episode_*.json` of each chosen teacher demo;
the corrections folder can be passed to 79 directly as a second folder (`79:53` takes several), so only the teacher subset needs
links. v2's names are unique (one folder, one seed per process, `run2_pc.sh:25-38`), so links cannot clash.

## Minor

- **r3-2** Reserve harvest trigger: "if fewer than ~150 survive, harvest seeds 5-6 on Tue" (3.1) uses a number known only after
  the Tue-eve bulk run + filter, but the reserve harvest must run before it and not at the same time (`82:28`). Decide it Tue
  midday from an estimate (failed trials x sorter pass rate per class), or drop it for round 1.
- **r3-3** D2 "include human corrections in run4 only if the pilot passes by Tue evening" cannot happen as scheduled: fingers,
  view and snapshot loading are Wed work (Wed row), and the run4 data freeze is Tue eve. Drop the option or say "Wed noon, before 79".
- **r3-4** Pass `--repo-id` to 79 (default `local/g1_volleyball_walkup_v1`, `79:33,55`) with the same value as the run4 case's
  `REPO_ID`; 80 takes the id from the map (`80:134`).
- **r3-5** Missing branch for the Mon gates: if the replay/on-off check fails, no harvest with capture on (else the harvested
  failures are not the policy's own); if the teacher's self-takeover is < 23/25, Tue becomes fixing `run_from_state` and run4
  follows the 1.1 "< 100" row.
- **r3-6** Mac rule on Tue: the sorter (Saad, "Mac", <= 50 trials) and the pilot (Cristina, "Mac + Vision Pro") are two sim
  processes; say they run on two different Macs, or not at the same time.

## Checked and correct

GPU: training needs 3.6 GB of the 6 GB (`train_smolvla.sh:15`); no row puts the server and training on the GPU at the same time
(harvest Mon/Tue, run4 Wed-Thu, eval Thu-Fri; the gate-fail branch records DART on the CPU and trains Thu). Disk: run1
checkpoints are 1.2 GB each (`du`, PROGRESS.md:1121), so run2 (9) + run3 (15) are ~29 GB; inferred free ~280 GB, the 150 GB
bar holds. Format: 79 checks fps 30, image 200x320x3, success, side file (`79:72-79`); 80 checks `handover_json`, waist, seed,
`stance_source`, timestep, variation, task in `TASKS` (`80:39-59, 116`); plan 3.2 lists all of them; `episode_corr_*` cannot clash
with `episode_s<seed>_t<trial>` (`69:189`). Stats: without `resume`, lerobot passes `dataset.meta.stats` into the normalizer and
unnormalizer (`lerobot_train.py:349-351, 375-377`), loaded from `meta/stats.json` (`io_utils.py:161-174`, `utils.py:93`), so the
freeze works as described. Snapshot holds pending queue and noise RNG (`table_env.py:258-289`). 82 flags `--no-walkup`,
`--from-handover`, `--teacher`, seed < 100 (`82:101-122`). Epoch base 1,227,666 / 1,691 = 726 frames.

## Not verified

PC free disk; harvest, eval, 79 and 80 run times on the PC; snapshot size; avp_stream `configure_video` without a device; Safari
during hand tracking; correction lengths; rented-GPU speed and price (all marked (U) in the plan and measured first).
