# Review: Level 1 v2 variation (plan humming-squishing-treasure A1-A7, B)

Reviewer: independent (did not write the code and has not seen the author's reasoning). Date 2026-11-01.
Scope: uncommitted changes in the canonical repo (11 modified files, 4 new: configs/randomization_v1.json,
requirements-record.txt, scripts/88_dr_image_checks.py, tests/test_integration_variation_v2.py).

## Verdicts

1. Did the code do what the spec said? **Yes.** A1-A7 are implemented as specified, B was run on the NEW config, and the two
   failing checks were reported with numbers, without loosening anything. No code bug found.
2. Was the spec right? **Not as it stands.** Two of the plan's decided numbers fail the plan's own acceptance bars:
   - G7 clearance with latency up to 100 ms: pooled p5 **3.47 cm**, bar 3.50 cm (min 3.18 cm >= 3.0 passes; pinky p5 3.51 cm passes).
   - Camera extremes with +-0.025 m / +-5 deg: worst margin **-12.5 px**, bar >= +10 px (the left palm leaves the head image).
3. Overall: **NO-GO for moving on to section C (smoke + v2 recording) with this config.** This is a plan decision for
   Finnegan, not a fix round: the code needs no change. Once the two numbers are decided (accepted as they are, or ranges
   changed by the user), the code side is GO.

## What I re-ran myself (logs under runs/l1v2_review_*)

| Check | Result | Log |
|---|---|---|
| 3 new tests (5 cases) + 3 existing vla_eval tests (69 sequence, seed-101 v1 handover vs T7, replay through the wire) | 8/8 pass | l1v2_review_tests.log |
| 3 v1 demos from the T7 (s101_t0000, s103_t0000, s104_t0075; latency 1, 3, 1 substeps) restored and replayed through the NEW apply, state compared every tick for all 726 ticks | bit-exact, 0 mismatches | l1v2_review_v1_replay.log |
| FIFO vs an independent model (which command is the target in every physics step), every latency L = 0..30 substeps, 6 ticks | 0 mismatches for all 31 values | l1v2_review_fifo.log |
| Snapshot mid-run at L = 25 with 2 commands pending -> JSON (handover_arrays) -> fresh TableEnv -> continue | bit-exact | l1v2_review_fifo.log |
| Texture round trip, walk scene, v2 config: variation -> JSON -> reset(variation); handover restore in a fresh env; nominal reset clears | matid equal in all three; ball and basket geoms never change; only table_top/floor change | l1v2_review_textures.log |
| First head frame after fresh-env handover restore, 6 starts each, v1 and v2 config | max diff 0 grey levels (both) | l1v2_review_image_restore.log |
| 88's projection vs segmentation + depth render, 3 starts x nominal + 2 extreme corners | every projected point hits the ball/basket or an occluder in front of it (rendered depth < point depth); basket floor depth matches within 3 mm -> projection convention is right | l1v2_review_projection.log |
| 69 one demo (seed 101, v2) -> 79 -> 80 | sentence "place the volleyball into the basket" (TASKS[1]) in npz, sidecar and LeRobot tasks.parquet; 80: 0 problems; sha256 in sidecar equals a recompute of the loaded config and differs from v1's; versions python 3.12.14, mujoco 3.13.0, mink 1.3.0, numpy 2.2.6, opencv-python-headless 4.13.0.92 (cv2 runtime 4.13.0) | l1v2_review_69.log, _79.log, _80.log |
| 82 --randomization configs/randomization_v1.json --teacher, 2 trials | 2/2; header records the path; textures {}, camera +-0.005, latency < 0.03; first start equals run1's seed-0 eval start bit for bit (dx_m -0.06243511408782288) | l1v2_review_82_v1flag.log |
| requirements-record.txt pins vs ~/dev/g1-ext/.venv | all 22 pins match the installed versions | (inline) |

## Gate logs read (builder's, all written after the last code edit 12:12:54)

All gate runs confirm the NEW config: walk JSONs show latency up to 0.0999 s, every trial textured, head offset up to 0.025 m,
tilt up to 4.98 deg; fixed-scene randomized runs also latency up to 0.098 s and textured; the 82 header names configs/randomization.json.

| Gate | Value | Bar | Status |
|---|---|---|---|
| G1 fixed nominal | 10/10, ACCEPTANCE PASSED | 10/10 | pass |
| G2 jitter 2 cm | 50/50 | 50/50 | pass |
| G3 randomized s0, s1 | 50/50, 50/50 | 50/50 each | pass |
| G4 distractors | 30/30 | 30/30 | pass |
| G5 walk s10-13 | 200/200 | >= 199/200 | pass |
| G6 walk s0, s1 | 100/100 | 100/100 | pass |
| G7 clearance pooled (300) | p5 3.47 cm, min 3.18 cm, pinky p5 3.51 cm | p5 >= 3.5, min >= 3.0, pinky p5 >= 3.5 | **FAIL (p5)** |
| G8 82 teacher seed 0 | 50/50, agree 50/50 | 50/50 | pass |
| G9 lift min | 8.36 cm | >= 8.0 | pass |
| G10 tests | before 300/511/10 pass, after 300/516/10 pass, 0 failures both | no new failure by name | pass |
| info s101-104 | 300/300, per-seed p5 3.51-3.58 cm | report | - |
| Camera extremes (88) | -12.5 px worst (start 0, centre:left_palm, offset [+,-,-] tilt [-,-,+]); 8 of 10 starts below 10 px | >= 10 px | **FAIL** |

Latency is the cause of the G7 miss (runs/l1v2_g7_latency.log): p5 3.71 cm at 0-33 ms, 3.54 cm at 33-67 ms, 3.38 cm at 67-100 ms;
corr(latency, closest) -0.24. Camera (runs/l1v2_camera_breakdown.log): offset alone min 10.5 px (passes), tilt alone min 5.2 px,
v1 range min 16.7 px; share of random draws under 10 px per start 0-12 %.

## Blocking findings (plan level, need Finnegan's decision; no code change)

B1. **G7 p5 3.47 cm < 3.50 cm with latency up to 100 ms.** Trigger: any v2 run; the 67-100 ms third of starts pulls the pooled p5
    down (3.38 cm there). Impact: the plan's clearance bar (FINGER-PLAN section 5) is not met; every pick-and-place still succeeds.
    The miss is 0.3 mm. Options are the user's: accept 3.47 cm, or change the latency range. The builder correctly did neither.
B2. **Camera extremes fail at +-0.025 m / +-5 deg.** Trigger: the worst combinations of all three tilt axes plus offsets; the left
    palm (lift/centre moments) leaves the head image by up to 12.5 px. Impact: a few % of v2 demos would have frames where a palm
    is out of view; the plan's own bar (>= 10 px inside) fails. Options are the user's (accept, or a smaller tilt range). Note the
    check measures "inside the frame", not visibility: in some starts the hand already hides the ball centre at handover,
    independent of this change.

## Minor findings

m1. scripts/82_vla_eval.py:166 header "prompt" is still policy_wire.TASK, but in --from-handover mode the trial now sends
    ep["task"] (82:225); the module docstring line 3 says the same. The report can name the wrong sentence for v2 demos.
m2. requirements-record.txt pins only distributions whose modules 69 imports on the Mac. Declared dependencies of mujoco
    (absl-py, etils, glfw, pyopengl) and huggingface_hub (fsspec, hf-xet, packaging) are installed unpinned by pip. pyopengl matters
    on the Ubuntu PC: MUJOCO_GL=egl rendering imports it, the Mac (CGL) never does, so import tracing could not see it.
m3. scripts/69 provenance() reads importlib.metadata.version("opencv-python-headless"). On a machine with only opencv-python,
    it raises PackageNotFoundError after the npz is already written (69:186 before 69:210), leaving an npz without sidecar that 79
    skips ("no sidecar"). cv2.__version__ would be robust. Only reachable if the PC env is not built from requirements-record.txt.
m4. Adjacent, outside the spec: scripts/83_open_loop_check.py sends policy_wire.TASK for frames of demos that carry another
    phrasing; tests/helpers/make_robot_day_data.py exits if dataset episode 0's task != TASK. With the plan's seeds 109-112, v2
    episode 0 is seed 109 trial 0 -> TASKS[4], so that helper would refuse a v2 dataset.
m5. scripts/79_convert_smolvla.py:5 docstring still says task "put the volleyball in the basket".
m6. Info: with textures on, the texture draw shifts every later random draw, so seed-0 held-out starts under v2 are different
    layouts, not only different looks. The fair run1 comparison must use --randomization configs/randomization_v1.json (verified to
    reproduce run1's start exactly).
m7. The numbers in l1v2_g7_latency.log and l1v2_camera_breakdown.log come from ad-hoc code not in the repo; the bucket label
    "67-110 ms" should read 67-100 ms.
m8. Test-only (not a NO-GO reason, user rule): test (a) asserts `env.latency == 0` (that is the shifted run, reads oddly) and covers
    only whole ticks (e.g. 15 substeps untested; my FIFO check covered 0..30); the vla_eval `recorded` fixture asserts the canonical
    sentence, which only holds while seed 100 trial 0 is the saved demo.

## Lens notes

- Latency FIFO: no off-by-one (independent per-substep model, L = 0..30), reset in _set_up on every reset and before
  restore_handover refills it, deterministic, survives snapshot/JSON/fresh env. <= 1 tick bit-identical: test (b) plus 3 real v1
  demos replayed bit-exactly for 726 ticks.
- Textures: choice is in the variation dict, restorable, geom_matid restored per apply (in FIELDS), ball/basket untouched, colour
  rule r <= g, b holds for every material (88 --textures). Old variations without "textures" load (`.get`).
- Wrist cameras: not recorded (episode.IMAGE_KEYS = ("head_image",); observation() renders only those; openpi wrist slots are
  empty), so the scalar camera change only matters for the head camera. Fine as specified.
- Config/flag wiring: 82 both load() calls and 69 take --randomization; v1 keeps the old values with textures off; v1 draws the same
  starts as before (seed-101 test vs T7, run1 start check).
- Prompt reaches the dataset: verified end to end (npz, sidecar, LeRobot tasks). 80 now accepts any of TASKS and checks npz == dataset.
- Provenance: sha256 of the loaded config + versions present and correct.
- Readability: small functions (_take_due_targets, _texture, task_for, provenance, add_dr_materials), named constants with source
  (MAX_LATENCY_TICKS, TEXTURE_SIZE, DR_MATERIALS, VERSIONED), why-comments present. No readability major.
- Mutations: the builder's three mutation logs show each new test failing once (clamp to one tick, one step late, matid not restored).
