# Week plan Mon 2026-11-16 to Fri 2026-11-20: correction demos (teacher first, then Vision Pro)

Written 2026-11-09 (before run3 ends and before the SAP visit). Revision 2 (2026-11-09) after plan review rounds 1 and 2
(`.planning/reviews/week-after-sap-plan.md`; what changed is listed in section 7). Status: **draft, needs Finnegan's rulings
in section 6.** Audience: Finnegan, Cristina, Saad. Owners below are **suggestions**. Nothing here was run: code facts come
from reading the files. Lines marked (R) were taken from the code research of 2026-11-09 and not re-read by me. Lines marked
(U) are unverified.

Words used a lot:
- **VLA**: our learned policy (SmolVLA). It looks at the camera image and outputs arm and hand commands.
- **Teacher**: the scripted grasp program `g1sim/ball_grasp.py`. It made all 1,691 training demos and succeeds 100/100.
- **Correction demo**: we let the VLA run in sim, stop it where it goes wrong, and record someone (teacher or human) finishing
  the grasp from that exact moment. The policy then learns how to get out of its own mistakes. This is called **DAgger**.
- **Snapshot**: a saved copy of the full sim state (physics, controller targets, noise generators). Restoring it puts the sim
  back to the exact same moment (`TableEnv.snapshot_handover` / `restore_handover`, `g1sim/table_env.py:258`, `:271`).
- **Epoch**: one pass over every frame of the dataset. Epochs = steps x 8 (batch size) / frames.
- **Share**: the part of the training samples that are corrections (e.g. 50 % = every second sample is a correction frame).
- **IK** (inverse kinematics): computes the joint angles that put a hand at a wanted position. **FK** (forward kinematics): the
  other way round, joint angles -> where the hand is.
- **mj_forward**: MuJoCo's "recompute positions and contacts without moving time". A snapshot calls it, which is why we check
  that taking snapshots does not change the eval (Mon row).
- **Side file**: the small JSON next to each demo npz (seed, variation, ...). The code calls it "sidecar".
- **Symlink**: a file entry that points to another file. A folder of symlinks costs almost no disk.
- **Arrow cache**: a decoded copy of the dataset that lerobot writes to disk on the first load; about as big as the dataset.
- **Warm-up / decay schedule**: the learning rate ramps up over the first 1k steps, then falls smoothly to near zero at the end.
- **Wilson interval**: the range a success rate probably lies in, given only 100 trials (e.g. 47/100 -> 38-57 %).
- **HG-DAgger**: a human watches the policy and takes over only when it goes wrong; only the takeover is trained on.
  **RaC**: the same idea, with an explicit "recover first, then finish" split. **Sirius**: keeps the policy's own frames but
  weights them low and the human's high. **IWR**: trains on a 50/50 draw of original demos and corrections.

## 0. Goal of the week and "done"

Goal: turn the VLA's own failures into training data, retrain once, and measure if it helped.
Cheap automatic path first (teacher takes over from saved failure states), then human corrections via Vision Pro for the
failures the teacher cannot fix. All in sim. This does **not** close the sim-to-real gap; it only makes the sim policy better.
Default (D2): **run4 is teacher corrections only**; the human corrections of this week are round-2 data (run5).

Done means all five:
1. **Failure states saved:** snapshots from >= 100 failed run3 trials (seed sets not used for eval, see 3.1), each with its
   event snapshots and the sparse ring (3.2), and a takeover point chosen per failure class.
2. **Teacher takeover measured:** takeover success rate per failure class on >= 50 saved states, written down.
3. **Vision Pro pilot works:** both arms + both hands driven from the headset in the `volleyball_walk` scene; >= 1 human
   correction episode passes converter 79 and validator 80 (with the 80 change of Tue). Stretch: >= 30 human corrections
   (round-2 data).
4. **run4 started on the Hive PC** by Wed evening (Thu evening in the "takeover gate fails" branch, 1.1), from run3's chosen
   checkpoint, on teacher demos + teacher corrections.
5. **Eval number:** run4 vs run3 on the same 100 held-out starts (`82 --no-walkup --seed 0`, snapshot capture off), compared
   seed by seed, with the three failure classes and the "passed before, fails now" count (forgetting).

## 1. Inputs that change priorities

### 1.1 Decision table (read on Monday morning, 1 h, whole team; the last row on Tuesday evening)

| If this happened | Then do this first | Effect on the week |
|---|---|---|
| SAP: fence held the arm near basket/stand on the real robot (the O1 case: robot leans forward like sim 85) | Fix the safety rulings O1/O2 (1.2) before any data work | Correction work slips 1-2 days; skip human recording, keep teacher path |
| SAP: safety stop, jerk or hit on the real robot that is **not** the O1 case | Stop. Find the cause first (robot log, replay the same commands in sim 85); no data work until Finnegan rules | Week becomes a safety week; teacher path only if time is left |
| SAP: real camera image or ready pose clearly differs from sim (ball size, colours, viewpoint, torso lean) | Update the sim scene/camera/randomization first, then **re-evaluate run3 in the new scene** (seed 0, 100 trials, PC GPU), then harvest failures | Corrections recorded in a wrong scene are wasted; without the re-eval run4 vs run3 is not like for like |
| SAP: real grasp looked like sim (tried, missed, no jerks, no hits) | Plan as written | none |
| SAP visit cancelled, or the robot was not run | Plan as written; O1/O2/wrist envelope stay open (1.2); no new sim-to-real facts | none for the data work |
| run3 eval >= 85/100 (our sim target) | Do teacher takeover only for the remaining failures; Vision Pro pilot becomes "build, don't record much" | More time for sim-to-real items from SAP |
| run3 eval 55-84 | Plan as written | none |
| run3 eval < 55 (no clear gain over run2's 47, given the +-10 point noise) | First find out why training did not help (data, loader, learning rate), not corrections; consider the rented GPU (4.3) | Corrections move to the week after |
| Vision Pro not available that week | Teacher path only; human corrections later | Done item 3 becomes "pilot code ready, tested with recorded hand data" |
| Hive PC down or busy | Rented GPU for run4 (4.3); harvesting needs a GPU too | Budget question for Finnegan |
| **Tue eve: teacher takeover gate fails** (< 50 % success on "never lifted") | Bulk corrections only for the classes that reach >= 50 %. If that gives **>= 100 accepted** corrections: run4 Wed as planned on them, steps from the D4 formula (e.g. 100 x 600 = 60k frames, 50 % share -> 22.5k steps, 11.5 h). If **< 100**: no run4 Wed; Saad adds action noise to the teacher recorder for DART (D8, new code, ~0.5 d (U)); DART demos recorded Wed night on the PC CPU (40-per-process batches); run4 starts Thu on takeovers + DART | Second case: done item 4 moves to Thu evening, eval to Fri/next Mon; Wed human recordings target "never lifted" (round 2) |

### 1.2 Safety items to carry over if not closed before SAP

- **O1** (`.planning/reviews/fence-smooth.md:228-234`): replay test E2 stays red because in 85 the torso leans forward and the
  fence holds the replay near the basket. Needs Finnegan's pick of 6.3 (a)/(b)/(c). Use the real waist pitch seen at SAP.
- **O2** (`.planning/reviews/fence-smooth.md:236-246`): the soft-exit route (`arm_paths.way_out`) is checked only against
  table/stand/basket, not the robot's own body; from poses near the torso it goes 2-6 cm into torso/head in FK. Fix in 87
  (check the route against the self groups) or hold + operator fault in 86.
- **Wrist envelope** (`.planning/reviews/fence-smooth.md:114`, m7): 18/100 teacher trials go above `ENVELOPE_HI` wrist pitch
  (`g1sim/real_safety.py:52`), so the real robot would clip them. Relevant here too: corrections must stay inside the same
  envelope (`g1sim/real_safety.py:48-52`) or the real robot cannot replay what the policy learns (filter in 3.4). The
  envelope is a fixed table of numbers in the code (derived once from v2's `stats.json`, `real_safety.py:47`); it does not
  move when the dataset changes, and we keep it fixed this week.
- Open rulings F1-F5, S1-S6, r3-m1, r3-m3 in `.planning/real-robot/SAFE-PATHS.md`, if still open.

## 2. Day by day

Order: snapshot capture + teacher takeover -> harvest -> sorter + 80 change -> bulk teacher corrections -> one new dataset
(79) + run4 -> eval. The Vision Pro pilot runs in parallel and feeds round 2.
Mac load is an estimate (U). Rule: say what runs on the Mac before starting; one sim process at a time; <= 50 teacher trials;
no VLA model on the Mac. The policy server and training both need the PC's GPU (6 GB), so they run **one after the other**:
harvest (Mon eve / Tue am) -> run4 (Wed-Thu) -> eval (Thu-Fri).
Same code everywhere: PC and Mac use the same repo commit and the same randomization config (82's `--randomization` default).
Snapshots taken on the PC (x86) and restored on the Mac (arm64) give the same start state, but bit-exact restore is only
promised on one machine (`table_env.py:272-273`); that is fine for a start state, because the recorded episode is then made
entirely on the Mac.

Before Monday (run3 ends Wed 2026-11-11 ~20:00, PROGRESS.md:55-56): the run3 eval and the D4 checkpoint pick run on the PC GPU
in the days before. At most **2 checkpoints** (the last and one earlier), 100 trials each. Time per 100-trial eval on the PC is
not known (U); if the pick is not done by Sunday, take the last checkpoint and do not spend Monday's GPU on it.

| Day | Task | Owner (suggestion) | Runs on | Mac load | Check that proves it works |
|---|---|---|---|---|---|
| Mon | Decision table 1.1, confirm run3 checkpoint | all | - | none | Written decision in PROGRESS.md |
| Mon | **Snapshot capture in the eval loop** (inside the trial loop of `run_policy_trial`, `vla_eval.py:345-401`): (a) **event snapshots** at the handover, the first robot-ball contact (own check: any robot geom touching the ball; this is not a tracker event) and the first tick of `lifted`, `carried`, `placing` (the tracker's events, `vla_eval.py:39`); (b) a **sparse ring** over the whole trial, one snapshot every 15 ticks (0.5 s), ~90 per 45 s trial. Each snapshot also stores tick, ball position and phase. Written out only for failed trials (a test flag writes them for every trial); off by default, on with a flag. Take it right after the image is rendered (mj_forward caveat (R)). ~0.5-1 d (R) | Finnegan | Mac (code), tiny test, **no model** | 1-2 cores (sim + 81 replay server), ~10 min (U) | (1) Replay gate on **one machine**: restore a saved snapshot, step 60 ticks with the same commands: joint positions match the original run bit for bit. (2) **On/off check without a model:** `81 --replay` + `82 --from-handover` on 5 teacher demos (the replay server serves the recorded actions, no torch, `81:3-5`), once with capture off, once on with "write for every trial". Pass = both runs pass 82's existing replay gate (`vla_eval.py:464-492`: state error 0.0, events and latch tick equal to the recording) and the executed actions, events and latch ticks are identical between off and on. Wall-clock fields (`wall_s`, `server_ms`, `round_trip_ms`) are ignored, they always differ. If capture is also added to `run_teacher_trial`, repeat with `82 --teacher --no-walkup --seed 0 --trials 5` |
| Mon | **Teacher takes over, first cut:** `run_from_state()` that starts from the current command instead of `env.ready_pose` (`ball_grasp.py:196`) and moves there smoothly with IK; lift check against the stand height instead of the start rest (R). Episode ends like a teacher demo (3.4) | Saad (Finnegan reviews) | Mac (25 + 25) or PC CPU (50 + 50) | 1-2 cores, 50 trials, ~15 min | Takeover from the teacher's own mid-grasp snapshots (no VLA needed): Mac 25 teacher runs make 25 snapshots, 25 takeovers, >= 23/25 succeed; on the PC CPU 50 + 50, >= 45/50 |
| Mon | **Vision Pro setup:** install the free "Tracking Streamer" app (visionOS 2.1+) and `avp_stream` >= 2.50 in a **separate** venv under `~/dev/g1-ext` ([PyPI](https://pypi.org/pypi/avp_stream/json), [app](https://apps.apple.com/us/app/tracking-streamer/id6478969032)). Same Wi-Fi as the Mac; bring a phone hotspot (venue Wi-Fi may block devices from seeing each other (U)) | Cristina | Vision Pro + Mac | 1 core, minutes | A 60 s log of `get_latest()` wrist poses + pinch at >= 30 new samples per second |
| Mon eve (or Tue am if capture slips) | **Harvest failures:** capture code on the PC, run3 checkpoint, `82 --no-walkup` with capture on, seeds 2, 3, 4 (100 starts each; seed 0 = eval, 1 = reserve, >= 100 = training demos, `scripts/82_vla_eval.py:10-11`; 82 refuses >= 100, `:121-122`) | Finnegan | Hive PC GPU | none | Snapshot files on disk, one row per failed trial with its class. First measure 10 trials to size the run (time per trial on the PC not known (U)) |
| Tue | **Phase sorter + takeover point per class:** pick the takeover snapshot by class (3.2), then the entry phase: ball untouched on stand -> open hands, back off, reach; ball held and lifted -> continue at the waist turn; ball on table/floor -> skip (R). Build on the teacher's own snapshots in the morning, use harvested states when they arrive. ~1 d (R) | Saad | Mac | 1-2 cores, <= 50 trials | Takeover success per failure class on >= 50 harvested states (done item 2; run on the PC CPU if > 50 trials). Gate: >= 50 % on "never lifted" -> teacher makes bulk corrections; else the last row of 1.1 |
| Tue | **80 change + disk + stats script** (3.5, 4.2): correction side files allowed by rule; `df -h` on the PC against the bar in 3.5; script that compares old and new normalization stats per channel. ~0.5 d (U) | Finnegan | Mac (code) | 1 core, minutes | 80 passes a 2-episode correction test set (seed 2, shared variation) and still fails a seed-50 episode without `source` and a teacher pair with the same variation |
| Tue | **Vision Pro pilot, two arms:** new `AVPPilot` with the same clutch as `PhonePilot` (hand follows only while pinching, relative to where the pinch started; `g1sim/teleop.py:190-214`); two `ArmIK`s as in `ball_grasp.py:182`; scene **`volleyball_walk`** as 82 uses it (`vla_eval.py:24`; a snapshot only restores into the same scene), start pose, `FORBIDDEN` (`ball_grasp.py:102`), success from `placement_checks` (`ball_grasp.py:138`), task from `policy_wire.TASKS` (`g1sim/policy_wire.py:37`). Today teleop is right arm only (`g1sim/teleop.py:25`) and 57 builds a default `TableEnv` with the bottle (`scripts/57_teleop.py:203`, `g1sim/teleop.py:281`) | Cristina (+ Finnegan for IK, after the 80 change) | Mac + Vision Pro | 2-3 cores while piloting (U) | Both robot hands follow both wrists, no jump when the pinch starts, speed caps from teleop.py:140-148 (R) hold |
| Tue | **Reserve harvest (only if needed, 3.1)** on seeds 5-6 with 82 on the PC GPU. 82 writes the scene XML at start (`82_vla_eval.py:28`): never start it while takeover or recorder processes run on the PC, unless the takeover script is checked not to read that file | Finnegan | Hive PC GPU | none | As the Mon harvest |
| Tue eve | **Bulk teacher corrections** from all harvested states the sorter accepts. On the PC CPU in **40-per-process batches under `runs/pc_watchdog.sh`** (a 69-style process leaks ~1.8 -> 3-4 GB over ~40 demos, PROGRESS.md:123-124). **Data freeze for run4 when this ends** | Finnegan | Hive PC CPU | none | Episodes with `success=True` + side file, counted per class, after the filter 3.4 |
| Wed am | **Build one run4 dataset + start run4** (3.5): pick the teacher subset (D1 share, fixed random seed, list saved); a folder of symlinks to those teacher npz + the accepted corrections; **79** on that folder; **80** on the new set; then **freeze stats** (write v2's `meta/stats.json` into the new set, 4.2; after 80, because 80's stats check compares against the npz frames, `80:96-102`); new `run4` case in `scripts/train_smolvla.sh` (from a checkpoint with a fresh schedule, not `resume`). Expected 79 ~10-30 min, 80 ~20-40 min (U). Target start ~14:00, latest 20:00 | Finnegan | Hive PC | none | 80: 0 problems on the new set; stats check written down; first 50 training steps logged with sane loss |
| Wed | **Pilot: fingers, waist, view, start from snapshot.** Pinch/finger curl -> one grip value 0..1 -> 6 Revo2 values, capped near the teacher's holding values (`ball_grasp.py:84-88`); waist yaw (index 26) by ruling D3; **view, first route:** avp_stream's own video channel shows our rendered 2D frames inside the Tracking Streamer app (`configure_video(...)` without a camera device + `register_frame_callback`, WebRTC; [repo](https://github.com/Improbable-AI/VisionProTeleop), not tried (U)); this is not the 3D scene upload we forbid (section 5). **Fallback:** 57's web server on 0.0.0.0 instead of 127.0.0.1 (`scripts/57_teleop.py:209`) and the 2D stream in the headset browser (not verified that Safari stays visible while the app tracks the hands (U)). Load a snapshot instead of `env.reset`, arms start from the current command (R) | Cristina + Saad | Mac + Vision Pro | 2-3 cores, sessions <= 45 min | One human correction from a saved failure state goes through 79 + 80 without errors (done item 3) |
| Wed pm | **Human recording session** (if the check above passed): failures the teacher cannot fix first (by the Tue per-class numbers, e.g. odd contacts, "pushed in without lift" if the teacher fails it). **Round-2 data**, not in run4 (D2) | Cristina, Saad | Mac + Vision Pro | 2-3 cores, 2 x 45 min | Accepted episodes counted after the filter 3.4 (stretch >= 30) |
| Thu | run4 trains (size by D4, 4.1; default 45k steps = 23 h: start 14:00 -> done Thu ~13:30; start 20:00 -> Thu ~19:30). Meanwhile: write up takeover numbers, more human recordings for round 2, LEARNING_PATH | all | Hive PC GPU; Mac light | 2-3 cores if recording | run4 log shows steady loss, checkpoints saved |
| Thu eve / Fri | **Eval run4** on `--seed 0`, 100 starts, same settings as run3's eval, **snapshot capture off**. Eval time on the PC is not known (U); if it runs into Fri afternoon, the review uses what is done | Finnegan | Hive PC GPU | none | Done item 5: success, per-class failures, paired run3 vs run4 table, forgetting count |
| Fri | Team review: keep run4? Round 2 (human data)? Update PROGRESS.md, LEARNING_PATH, START_PROMPT | all | - | none | Decision written down |

## 3. Data plan

### 3.1 How many
- Literature: each correction round adds about 20 % (HG-DAgger, [paper](https://ar5iv.labs.arxiv.org/html/1810.02890)) to
  one third (Sirius, [paper](https://arxiv.org/html/2211.08416v3)) of the original data.
- Length of one correction: it must end like a teacher demo (3.4). A teacher demo runs from the handover to the end of
  `face_front` + 30 ticks and is 600-800 frames, 726 on average (`scripts/69_volleyball_demo_factory.py:8-11`;
  1,227,666 / 1,691). A takeover from "never lifted" or before a forbidden contact replays almost the whole sequence (face,
  reach, touch, squeeze, lift, turn, centre, release, retract, face_front; `ball_grasp.py:204-252`), so plan with
  **~600 frames per correction** (500-700, plus 1-2 s of back-off (U)). Measure the real lengths on Tue.
- Our target for round 1: **150-300 teacher corrections** -> ~90k-180k unique frames.
- Source: run2's failure mix was 23 never lifted, 10 failed the lift check (ball pushed into the basket without a lift),
  16 forbidden contacts per 100 (PROGRESS.md:48). 300 harvested starts with ~40 % failures gives ~120 failed trials x 1-3
  takeover points each (U, depends on run3). Losses on the way: classes the sorter skips, takeovers that fail, and the
  envelope filter (3.4): 18/100 teacher trials already go above the wrist limit, so expect it to drop some teacher
  takeovers too. If fewer than ~150 survive, harvest the reserve seeds 5-6 on Tue (GPU free until Wed).
- Seed sets: harvest on seeds 2-4 (5-6 as reserve). Never seed 0 (eval) or 1 (reserve); >= 100 are the training demos
  (82:10-11).

### 3.2 How a correction is recorded
- Takeover point per class, chosen from the saved snapshots:
  - **Forbidden contact:** the ring snapshot 1-3 s before the contact tick.
  - **Never lifted** (stuck after the handover, PROGRESS.md:48): the ring snapshot ~1 s before the first robot-ball contact
    (own contact check, Mon row); with no contact, the one 2 s after the handover. The last 4 s of the trial are only the
    stuck pose and are useless.
  - **Failed the lift check** (reached `placing` with the ball in the basket, never `lifted`, then timed out at 45 s;
    `vla_eval.py:13-16`, `:105`): the ring snapshot 1-3 s before the ball first left its rest on the stand. The end of the
    trial is a ball that has lain in the basket for a long time; nobody can correct from there.
- Record only the takeover segment, as in HG-DAgger and RaC ([RaC](https://arxiv.org/html/2509.07953v1)): first a short
  recovery back to a normal pose (e.g. above the ball), then the finish of the grasp and place, ending like a teacher demo.
  The VLA's own frames before the takeover are **not** used (Sirius gives them weight 0).
- Format identical to a teacher demo: `head_image` 200x320, `state` and `actions` 27-D, 30 fps, `success`, the handover
  snapshot in the npz (`handover_json`), file name `episode_*.npz` (79 picks up only that pattern, `79:45-48`)
  (g1sim/episode.py:5-8, :42-68 (R)); task string from `policy_wire.TASKS` (today teleop writes openpi's `DEFAULT_TASK`,
  which 82 rejects (R)). Correction file names must not clash with teacher names: 79 sorts by file name only (`79:47-48`),
  e.g. `episode_corr_s2_t017_k1.npz`.
- **Side file fields that 80 requires** (`scripts/80_validate_dataset.py:42-56`; teleop's side file has none of the last
  three, `g1sim/teleop.py:298-301`): `seed` (int; 2-99 for corrections after the 80 change), `variation` (copied from the
  snapshot), `stance_source` (`"provisional"`, as the teacher demos), `timestep` (1/300). New: `source`
  (`teacher_takeover` / `human`), `failed_trial` (seed + trial of the VLA trial), `takeover_tick`, `snapshot_id`, `class`.
- Domain randomization is kept for free: the snapshot carries the randomized start of that trial (CLAUDE.md rule 3).

### 3.3 Weighting
- lerobot 0.6.1 has no per-episode weight (only `rabc`, `uniform`; utils/sample_weighting.py:125-132), and training on
  several datasets at once is switched off (`datasets/factory.py:124`). An episode listed twice in `--dataset.episodes`
  still counts once (yes/no mask, `datasets/sampler.py:81-84`), so "corrections x 2" needs physical copies.
- **Option B (recommended, no copies):** one new dataset = all accepted corrections x 1 + a random teacher subset sized to the
  share. Teacher frames = correction frames x (1 - share) / share.
  - **50 % share (IWR, default in D4):** 200 corrections x 600 = 120k frames + 120k teacher frames (~165 demos at 726) = 240k.
  - 33 % share: 120k + 240k teacher frames (~330 demos) = 360k.
  - Fewer teacher starts in the mix is the price (more at 50 %): watch the forgetting count in the eval.
- **Option A (heavy, not this week):** all 1,691 teacher demos + corrections copied K ~ 4-5 times (30 % needs
  K = 0.43 x 1,227,666 / 120k = 4.4). It needs either the merge (3.5: full copy of v2 + Arrow caches, ~400-590 GB, does not
  fit the PC disk) or re-converting all 1,691 + ~880 corrections incl. copies with 79 (~1.75M frames: ~240 GB dataset +
  ~240 GB cache), plus a `copy_of` rule
  in 80 because copies share their variation.
- Not this week: patching our own weighter into lerobot (third-party edit).

### 3.4 Quality filter (an episode is dropped if any is true)
- `success` is false by `placement_checks` (ball_grasp.py:138), or any contact with `FORBIDDEN` (ball_grasp.py:102).
- Any command outside the demo envelope `ENVELOPE_LO/HI` (`g1sim/real_safety.py:48-52`): the real robot would clip it.
- It does **not end like a teacher demo**: retract to the ready pose, waist yaw back to 0 (`face_front`), then 30 ticks
  still. Otherwise the policy learns to stop at the basket.
- Longer than 30 s (teacher demos are at most ~27 s), or more than 2 s with no motion (trim idle frames before and after).
- Human episodes with a visible jump (> 0.35 rad between ticks) when the clutch starts.

### 3.5 Validator 80 and the run4 dataset
- Today 80 rejects every correction: seed < 100 fails (`80:50-51`), and two episodes with the same `variation` fail
  (`80:53-56`); 1-3 takeovers from one failed trial share it.
- **80 change (Tue, Finnegan):** a side file with `source` in (`teacher_takeover`, `human`) may have seed 2-99 (never 0 or 1)
  and may share its variation with other corrections of the same `failed_trial`; every other episode keeps today's rules.
  That is the only 80 change.
- **Route (Wed am):** make a folder of symlinks to the chosen teacher demos (D1 share). Link **both** files of each demo,
  `episode_*.npz` **and** its side file `episode_*.json`: 79 looks for the side file next to the link (`79:78`, writes the link
  path into the map `79:87`; 80 does the same, `80:45`), so npz-only links fail every episode with "no sidecar" (review r3-1).
  Pass the corrections folder to 79 as a second folder (79 takes several, `79:53`) and pass `--repo-id` with the same value as
  the run4 `REPO_ID` (79's default is `local/g1_volleyball_walkup_v1`, `79:33,55`; review r3-4). 79 globs `episode_*.npz` (`79:47`). 79 writes one dataset and one `<root>.convert.json` (`79:98`), which 80 reads
  (`80:128`), so 80 works unchanged apart from the seed/variation rule. Training then needs no `--dataset.episodes`.
  Keep the symlink folder and the source npz until the run4 eval is done (80 and the eval read the npz through the map).
- **Why not the lerobot merge:** it writes a full copy of v2 + corrections (~195 GB), reads every source file into a new
  Arrow cache (`aggregate.py:620-621`), then loads the whole merged set and builds its cache too (`dataset_tools.py:305`):
  ~400-590 GB and hours of IO on a disk that already showed 76 % iowait (PROGRESS.md:71). Review round 2, N1.
- **Disk numbers** (frame sizes measured on v1: npz 35.1 GB / 217,800 frames = ~161 KB, dataset 30 GB = ~138 KB, Arrow
  cache ~30 GB = ~138 KB; PROGRESS.md:1151-1157):

  | Item | 50 % share (240k frames) | 33 % share (360k frames) |
  |---|---|---|
  | Correction npz (120k frames, written Tue) | ~19 GB | ~19 GB |
  | Teacher npz (symlinks, no copy) | ~0 | ~0 |
  | New dataset (79) | ~33 GB | ~50 GB |
  | Its Arrow cache (first training load) | ~33 GB | ~50 GB |
  | **Total new** | **~85 GB** | **~120 GB** |

  Free now, inferred (U): 849 GB before v2 (PROGRESS.md:119) - v2 npz ~198 GB - v2 dataset ~170 GB - v2 Arrow cache ~170 GB
  = **~310 GB**, minus run2/run3 checkpoints (not counted). **Bar: `df -h` on Tue must show >= 150 GB free** (120 GB worst case
  + room for snapshots and run4 checkpoints). If less: ask Finnegan to delete v2's Arrow cache after run3 has ended (it is
  only a cache, rebuilt in ~33 min if v2 is ever trained again; the eval does not need it). Moving files to the T7 is not a
  route: the PC does not see the T7 (PROGRESS.md:122; not re-checked).
- **Not measured** (check on Tue/Wed, write the numbers into PROGRESS.md): PC free disk; size of run2/run3 checkpoint
  folders; size of one snapshot and of all harvested snapshots; correction npz size per frame (assumed equal to a teacher
  demo); 79 and 80 run time on the PC for ~300-500 symlinked npz (Mac: 300 demos in 7 min, PROGRESS.md:1154); Arrow cache
  size and build time for the new set; whether the T7 is reachable from the PC now.

## 4. Training plan (Hive PC)

### 4.1 Time and epochs
- Speed ~1.84 s/step at batch 8 (run3 measured, PROGRESS.md:55, `train_smolvla.sh:38`). Loading took ~33-35 min for the
  1.23M-frame v2 (PROGRESS.md:71); the new set is 5x smaller, so expect ~10 min (U).
- Project rule (PROGRESS.md:51-53, from run2's 0.59 epochs -> 47/100): **size training in epochs, target 1.5-2.** The rule
  applies to the corrections: they are new states. (The teacher frames were already seen ~1.57 times in run2 + run3.)
- Formula: **steps = 1.5 x correction frames / share / 8.** With 120k correction frames:

  | Choice | Mix frames | Steps | Passes over the corrections | Time (+ ~10 min load) | Done if started Wed 14:00 / 20:00 |
  |---|---|---|---|---|---|
  | **50 % share, 1.5 passes (default)** | 240k | 45k | 1.5 | 23.0 h | Thu ~13:30 / ~19:30 |
  | 33 % share, 1.5 passes | 360k | 67.5k | 1.5 | 34.5 h | Fri ~01:00 / ~07:00: does not fit the week |
  | 50 % share, 30k steps | 240k | 30k | 1.0 | 15.3 h | Thu ~05:30 / ~11:30 |
  | 33 % share, 30k steps (old plan) | 360k | 30k | 0.67 | 15.3 h | Thu ~05:30 / ~11:30 |

  Math: 45k x 8 = 360k samples = 1.5 x 240k; half of them corrections = 180k = 1.5 x 120k. 33 % share: 1.5 x 120k / 0.333 =
  540k samples = 67.5k steps x 1.84 s = 34.5 h. 30k x 8 = 240k samples; at 33 %, 80k are corrections = 0.67 passes.
- Re-compute on Wed am with the real correction count (e.g. 150 x 600 = 90k at 50 % -> 33.75k steps, 17.3 h).
- Keeping teacher demos in the mix is the standard defence against forgetting (HandITL, [paper](https://arxiv.org/html/2605.15157v2);
  [LeRobot docs](https://huggingface.co/docs/lerobot/hil_data_collection)).

### 4.2 How to start
- From run3's chosen checkpoint: `--policy.path=<run3>/checkpoints/<step>/pretrained_model` with a fresh warm-up/decay
  schedule (warm-up 1k, decay over the run), copied from the `run3` case in `scripts/train_smolvla.sh:34-41`. The `run4`
  case sets `REPO_ID` and `DATASET` (`--dataset.repo_id` / `--dataset.root`) to the new set, `steps` and `decay` from D4,
  and its own `HF_DATASETS_CACHE`.
- Not `resume`: it reloads the old config, keeps the old normalization stats and the old sampler position (R).
- **Normalization stats:** with `--policy.path` and no `resume`, lerobot replaces the checkpoint's stats with
  `dataset.meta.stats` of the new dataset (lerobot_train.py:350-351, 375-377). SmolVLA normalizes state and action by mean
  and std (`policies/smolvla/configuration_smolvla.py:35-36`), so new stats shift every input and output of the run3 weights.
  The new set's stats come from its own mix, so they will differ from v2's.
  **Default: freeze them.** Order on Wed am: (1) 79, (2) 80 on the new set (its stats check compares the stored stats with
  the npz frames, `80:96-102`, so it must run before the freeze), (3) compare per channel the stats inside run3's checkpoint,
  v2's `meta/stats.json` and the new set's `meta/stats.json` (expect run3 = v2), (4) write v2's stats into the new set's
  `meta/stats.json`, so run4 normalizes exactly like run3. After the freeze, 80 on this set fails its stats check by design;
  keep the 80 log from step 2. The demo envelope is fixed constants (`real_safety.py:47-52`) and stays as it is. Ruling D9.

### 4.3 Rented-GPU fallback
- Use it if the PC is down, run3's result needs a re-run, or run4 + round 2 do not fit the week.
- Speed and price on a rented card are not measured for our setup (U). Data upload: only the new run4 set, ~33-50 GB.
- Same script with `DEVICE=cuda V2_ROOT=... HF_HUB_CACHE=... HF_DATASETS_CACHE=...` (scripts/train_smolvla.sh header).
- Budget needs Finnegan's OK (D6).

## 5. Risks and what we do NOT do

| Risk | Mitigation |
|---|---|
| Teacher cannot back off from odd poses (IK guards only forearm-table and arm-body, grasp_ik.py:76-80) | Measure per class Tue; give the teacher only the classes where it works, humans the rest (round 2); gate branch in 1.1 |
| Harvesting is slower than expected (time per trial on the PC unknown) | Measure 10 trials Mon; cut to 2 seed sets if needed |
| Snapshot capture changes the eval itself (mj_forward) | Model-free on/off check Mon (81 replay); capture always off for seed-0 evals |
| Policy server and training fight for the 6 GB GPU | Strict order: harvest Mon/Tue, train Wed/Thu, eval Thu/Fri |
| 82 rewrites the scene XML while other sim processes read it | Never start 82 while takeover/recorder processes run on the same machine |
| Too few corrections survive the filter | Reserve seeds 5-6 on Tue; teacher subset in Option B shrinks with them (share rule); < 100: 1.1 last row |
| PC disk too small | `df -h` Tue, bar >= 150 GB; delete v2's Arrow cache with Finnegan's OK (3.5) |
| Wednesday too full | run4 is teacher-only (D2); human data goes to round 2; 80 change on Tue; one 79 run instead of a merge |
| Human and teacher styles clash and hurt training ([paper](https://arxiv.org/pdf/2210.08073)) | Tag `source`; run4 teacher-only by default (D2) |
| Vision Pro problems: app stuck on "Initializing", stale "Connected", hands lost when out of view ([issues](https://github.com/Improbable-AI/VisionProTeleop)) | Hotspot, restart app, keep hands in view; KeyPilot as backup to finish an episode |
| Headset view feels bad because recording runs slower than real time (scripts/57_teleop.py:9-10 (R)) | Accept for round 1; slower, calm moves; short sessions |
| **avp_stream's 3D view uploads our scene zip over plain HTTP to a third-party server** when the macOS converter is missing (streamer.py:3756-3765 (R)) | Never call `configure_mujoco` / `update_sim` this week; 2D video channel or browser only |
| Operator fatigue, sloppy demos | Sessions <= 45 min, filter 3.4 |
| Gain too small to see: +-10 points noise at 100 trials (Wilson) | Paired comparison per seed; claim a gain only above ~15 points |
| Mac overheats | One sim at a time, <= 50 teacher trials, all heavy jobs on the PC |

Explicitly **not** this week: real-robot teleop (`unitreerobotics/xr_teleoperate`, waits for robot access); Qwen task
sentence; MediaPipe camera tracking (only if a teammate has no headset); the 3D AR scene in the headset; fine finger
retargeting (dex-retargeting); RL (HIL-SERL, pi*0.6/RECAP); patching lerobot; the lerobot merge; any training or VLA model on
the Mac.

## 6. Open decisions for Finnegan

- **D1 weighting:** Option B (one new set: corrections x 1 + a teacher subset, recommended; fits the disk, ~85-120 GB) or A
  (all teacher demos + corrections copied K ~ 4-5 times: needs the merge or a full re-convert, ~480-590 GB, does not fit the
  PC disk this week, plus the `copy_of` rule in 80). The share is part of D4.
- **D2 human data in run4:** **teacher-only run4**; human corrections go into run5 (round 2). They cannot make run4: the
  fingers, headset view and snapshot loading are Wed work, after the run4 data freeze (review r3-3).
- **D3 waist yaw in the pilot:** follow the operator's head yaw, a gesture/button, or a scripted turn like the teacher (+35 deg
  grasp, -35 deg place (R)). Suggestion: scripted turn on a button (least to learn for the operator).
- **D4 run4 size, share and start** (table 4.1): (a) **50 % share, 1.5 passes = 45k steps, 23 h (recommended**: keeps the
  epoch rule, IWR's ratio; more forgetting risk, ~165 teacher demos); (b) 50 % share, 30k steps = 1.0 pass, 15.3 h;
  (c) 33 % share, 30k steps = 0.67 passes, 15.3 h: a deliberate exception to the epoch rule, and then a weak run4 eval does
  **not** prove that corrections do not help. 33 % at 1.5 passes (34.5 h) does not fit the week. Steps are re-computed on Wed
  with the real correction count. Also: which run3 checkpoint (at most 2 evaluated, before Monday; else the last).
- **D5 owners:** who keeps the Vision Pro and who owns the pilot code (suggested Cristina + Saad) vs teacher takeover (Saad).
- **D6 rented GPU:** yes/no and budget, and the trigger (PC down, or run3 < 55).
- **D7 safety carry-overs:** O1 pick (a)/(b)/(c), O2 fix in 87 or 86, wrist envelope (raise `ENVELOPE_HI` or retrain), if
  still open after SAP.
- **D8 DART instead of / before takeover:** adding noise to the teacher's actions while it records gives recovery data with no
  VLA in the loop ([DART](https://proceedings.mlr.press/v78/laskey17a.html)). Cheaper, but does not target the VLA's actual
  mistakes. Suggestion: plan B when the Tue gate fails and fewer than 100 corrections are accepted (1.1 last row).
- **D9 normalization stats:** freeze run3's (v2) stats for run4 (recommended), or let lerobot use the new set's stats.
- **Disk (with D1):** OK to delete v2's Arrow cache after run3 ends if `df -h` shows < 150 GB free (3.5).

Not verified at all: `avp_stream` install and API in our venv (incl. `configure_video` without a device), rate/latency on our
Wi-Fi, whether Safari stays visible during hand tracking, PC eval time per trial, GPU run-to-run determinism of SmolVLA (not
needed for any check in this plan), measured correction lengths, rented-GPU speed and price, and the disk items in 3.5.

## 7. Changes

Revision 2 (review round 2):
- N1: merge dropped. Option B = one new dataset from a symlink folder (teacher subset + corrections) converted by 79; 80 needs
  only the seed/variation change (the list-of-maps change is gone). Disk table, >= 150 GB bar, v2 Arrow cache as the space
  lever (T7 is not seen by the PC), unmeasured list (3.5). Option A marked as not fitting the disk (3.3, D1).
- N2: Mon on/off check now model-free (`81 --replay` + `82 --from-handover`, plus `82 --teacher` if capture is added there),
  on one machine; compares actions, events and latch ticks, not wall-clock fields.
- N3: epoch rule applied to the corrections; formula, 4-row table and math (4.1); default 50 % share, 45k steps; choice in D4.
- N4: Tue "takeover gate fails" row in 1.1 (>= 100 accepted -> smaller run4 Wed; < 100 -> DART Wed night, run4 Thu).
- Minors: m-a Vision Pro video channel first, browser as fallback; m-b 80 before the stats freeze; m-c 82 scene-XML rule
  (Tue reserve harvest row, risk row); m-d same commit/config, replay gate on one machine; m-e re-eval run3 after a scene
  update; m-f `80:128`, run4 case passes the new `repo_id`/`root`; m-g jargon lines (side file, symlink, Arrow cache, schedule,
  share). Also: "first robot-ball contact" gets its own contact check (round-2 note on F4).

Revision 1 (review round 1): F1 80 change + side file fields; F2 merged-set check (replaced in revision 2); F3 mask sampler,
no "x 2"; F4 event snapshots + 0.5 s ring, takeover point per class; F5 run4 teacher-only; F6 ~600 frames per correction,
"ends like a teacher demo"; F7 stats freeze (D9); m1-m9 (1.84 s/step, 25 + 25 / 50 + 50 trials, on/off check, scene
`volleyball_walk`, 40-per-process batches, SAP rows, at most 2 checkpoint evals, envelope loss, jargon).
