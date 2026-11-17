# Track E1 - round 1

Reviewer: independent code reviewer (did not write Track E1, saw no author reasoning). 2026-10-18 17:51-17:58.
Reviewed against PLAN.md rev 6.2 (Track E1 task, sections 0, 2.7, 3.6) plus the round-3 plan findings that reach E1 (R3-1 own HF
cache, R3-6 firewall / r5-4). PLAN.md was being rewritten to rev 6.3 by the planner while this review ran.

Files: scripts/setup_robot_laptop.sh, scripts/89_robot_day_selftest.py, tests/helpers/loopback_lowstate_pub.py,
tests/helpers/make_robot_day_data.py, tests/data/{selftest_episode,replay_demo}.npz, docs/robot_day/reference_head_frame.png,
docs/ROBOT_DAY_RUNBOOK.md, tests/test_unit_runbook.py, scripts/81_smolvla_server.py (one line).

## What I ran (all output in full under runs/)
- Unit tests, sim env: `$PY -m pytest tests/test_unit_runbook.py tests/test_unit_policy_wire.py -v -rs` -> 37 passed, 1 skipped
  (86 not built yet) - runs/review_e1_r1_unit.log. Project .venv: 6 passed, 3 skipped with reasons - runs/review_e1_r1_unit_projectvenv.log.
- My own mutations on the loopback helper (copies in the scratchpad, repo file untouched) - runs/review_e1_r1_mutation.log:
  M1 non-loopback iface accepted -> tests still PASS (survived); M2 bound-interface assertion removed -> tests still PASS (survived);
  M3 mid-run robot-network result ignored -> test FAILS (caught). The author's mutations (runs/real_e1_mutation.log) also hold.
- 89's checks that need no DDS, in the fresh robot env (Python 3.10): versions, camera, bringup (84 --dry-run), imports all PASS -
  runs/review_e1_r1_89_nodds.log.
- 89 without the robot env (stock macOS python3): ModuleNotFoundError traceback, no CHECK line - runs/review_e1_r1_89_nosource.log.
- Read-only: GitHub release `robot-day-2026-10-18` has both assets, uploaded, sha256 file holds the bare file name (works on any
  laptop); repo is PRIVATE (operators need collaborator access for `gh repo clone`). Prefix HF cache size measured with `du`.
- Author's evidence checked: SETUP robot-env/release/policy-env OK on the fresh prefix; 81 --self-test on CPU from the prefix with
  `HF_HUB_CACHE=<prefix>/hf_cache/hub` -> SELF-TEST PASS (R3-1 satisfied); data files < 1 MB, replay starts/ends at BALL_READY_POSE.

## Not run (named, not passed)
- Full 89 with its DDS check (loopback helper really publishing, 84's readers receiving, the "selected interfaces:" parse against a
  successful CycloneDDS trace): pending the main session's push-1 check by plan design (section 0: no loopback DDS inside track
  builds; R3-4 one DDS run at a time). I did not start DDS.
- Ubuntu x86_64 path: `uv pip --torch-backend auto` CUDA pick, `gh` from apt (22.04 ships gh 2.4; `--clobber` support not verified),
  CycloneDDS discovery on Linux `lo` (not multicast-capable by default). No Linux machine here.
- macOS firewall steps: the firewall is off on this Mac, so the A4 instructions could not be exercised.

## Verdict 1 - did the code do what the plan said?
Yes, with one exception in substance (E1-1). Every E1 item exists: data script in $SV with the camera1 PNG -> head_image mapping,
the three data files, setup with the shared checksum+unpack, uv Python 3.10, own HF cache + refs/main pointer, 89 with guard first,
robot-network refusal, loopback helper, fake_camera path, 84 --dry-run, import check, policy check passing HF_HUB_CACHE; 81 cuda
line; runbook page 1 rules (caffeinate / systemd-inhibit, never box, gantry definition, release box), Part A, Part B with abort rules,
R11 placement, Part C stub; four planned tests plus a fifth (mid-run stop), all with Proves/Catches docstrings, importorskip in place.

## Verdict 2 - is the plan right for real hardware / the operators' laptops?
Mostly. Two plan gaps: the plan's test list leaves two of the helper's three loopback refusals untested (E1-2), and the plan's size
and time numbers (2.7 GB backbone, 30-45 min setup, 45 min Monday fallback) do not hold for a full-repo snapshot download (E1-1).

## Findings

**E1-1 (major) - the policy-env setup downloads 5.5 GB of unused ONNX files; real download ~10 GB, not "4-5 GB".**
- Trigger: `bash scripts/setup_robot_laptop.sh --policy-env` (runbook A3) on any operator laptop.
- Where: scripts/setup_robot_laptop.sh:96-101 calls `snapshot_download(repo, revision=rev)` with no allow/ignore patterns.
- Evidence: `du -shL` on the fresh prefix: SmolVLM2-500M-Video-Instruct snapshot 7.4 GB, of which `onnx/` = 5.5 GB; smolvla_base
  872 MB (incl. a gif and a notebook); prefix hf_cache 7.5 GB. Finnegan's working cache that 81 has always used is 2.7 GB (no onnx).
  The script header says "2.7 GB SmolVLA backbone", the runbook A3 says "about 30-45 min, 4-5 GB download"; real total is about
  8.3 GB backbones + 0.9 GB model + torch (1-4 GB, more with CUDA on Ubuntu).
- Impact: on home/hotel Wi-Fi the Sunday install takes hours instead of 45 min, and the Monday 08:00 fallback (plan 6.1: "budget
  about 45 min") cannot finish before 09:00; a small SSD can fill up. The tonight check passed only because this Mac downloads fast.
- Fix: pass `ignore_patterns=["onnx/*", "*.gif", "*.ipynb"]` (or an allow list of what 81 loads) to `snapshot_download`, correct
  the size/time numbers in the script header and runbook A3, then re-run `--policy-env` into a NEW prefix and the CPU
  `81 --self-test` from it (proves the trimmed offline cache still loads).

**E1-2 (minor) - two of the loopback helper's three refusals have no test (mutations survived).**
- Where: tests/helpers/loopback_lowstate_pub.py:40 (`not is_loopback(iface)`) and :119 (`bound != [args.iface]`); plan 3.6 lists
  both. Mutations M1 and M2 pass the whole test file (runs/review_e1_r1_mutation.log).
- Impact: low for the robot (the robot-network check, which is tested, refuses whenever 192.168.123.x exists), but a later edit can
  silently drop "loopback only" or the bound-interface assertion. Also `bound_interfaces` has never been run against a successful
  trace line, so a parse mismatch would make 89's DDS check always FAIL at the push-1 check (fail-safe, but a lost evening).
- Fix: `test_loopback_pub_refuses_non_loopback_iface` (pure `start_problem("en0", False)`) and
  `test_bound_interfaces_reads_selected_line` with a CycloneDDS line of the form
  `selected interfaces: lo0 (index 1 ...)`; confirm the real line in /tmp/cdds.LOG during the push-1 check.

**E1-3 (minor) - runbook hint for the most likely mistake is wrong.** A5 says "`CHECK versions: FAIL` - you forgot the source line".
Without the env, stock macOS has no `python` (command not found) and `python3` crashes with `ModuleNotFoundError: numpy` before any
CHECK line (runs/review_e1_r1_89_nosource.log). Fix: add "If you see `command not found: python` or `ModuleNotFoundError`: run the
`source` line, then 89 again" (optionally a try/except at 89's first import that prints the same).

**E1-4 (minor) - Monday 08:00 block runs 89 with the policy check on both laptops.** The laptop without the policy env gets
`CHECK policy: FAIL` and `SELFTEST: FAIL`. Fix: add "(laptop without the policy env: add `--skip-policy`)" under that block.

**E1-5 (minor) - re-running the setup re-downloads the 865 MB model every time** (`gh release download --clobber`, line 144), while
the runbook promises "finished blocks are quick". Fix: skip the download when the tar and its .sha256 already verify.

**E1-6 (minor) - firewall steps incomplete / unverified.** A4 adds `~/g1-robot-day/robot-env/bin/python`, which is a symlink into
`~/.local/share/uv/python/...` (checked); whether the macOS firewall honours the link is unverified. No step covers the laptop that
serves 81 to the other laptop (incoming TCP 8765; the ufw rule only allows 192.168.123.0/24). R3-6 asked for
`/usr/libexec/ApplicationFirewall/socketfilterfw --getglobalstate`. Fix: add that command, allow the resolved python path
(`readlink` of the venv python), and a one-line allow for 8765 on the R11 server laptop.

**E1-7 (minor) - clean-code readability.**
- tests/test_unit_runbook.py:124-130: `sleep=lambda dt: now.__setitem__(0, now[0] + dt)` and `next(net, True)` are tricks a
  second-year student will not explain; use a tiny `FakeClock` class with `now()` and `sleep(dt)`.
- scripts/setup_robot_laptop.sh: `${backend[@]+"${backend[@]}"}` (empty array under `set -u` on macOS bash 3.2), `${repo/\//--}`,
  `${spec%@*}`, `${2:+ $2}` need a one-line comment each.
- scripts/89_robot_day_selftest.py: `-> tuple` should be `-> tuple[str, str]` (status, detail); `last_line` has no docstring;
  the conditional lambda inside the `checks` dict (178-181) reads better as a small `policy_check()` function;
  `DRY_RUN_TIMEOUT_S = 60.0` has no source comment.
- tests/helpers/make_robot_day_data.py:96: `fps = 30` is a hidden constant inside main; read `meta/info.json` (it says 30) or make it
  a named module constant with its source.
- The flags test only reads fenced code blocks: inline commands in prose (A3 `--robot-env`, A5 `--skip-policy`) and `--steps`
  values are not checked (all currently correct by inspection against 84's STEPS).

**Note (no finding):** safety design of the read-only kit is sound: 89 installs the publisher guard before any other import, refuses
DDS while 192.168.123.x exists, the only publisher is a separate bounded process that re-checks the robot address every second
(tested, mutation-checked), and 89 passes the prefix HF cache explicitly (R3-1). The runbook's never box, gantry definition and
release box match plan section 7.

## Gate
Track E1 - round 1: **NO-GO** (1 major: E1-1 setup download size/time; 6 minors). E1-1 is a small fix (one argument + numbers +
one fresh-prefix re-run). Full 89 DDS run and the Ubuntu path stay "not run" (push-1 check / no Linux machine).

## Track E1 - response (fix round 1, implementer)
All 7 findings fixed; none refuted. Evidence under runs/ (`*_r2.log`).
- **E1-1 (major) fixed.** `setup_robot_laptop.sh` passes `ignore_patterns=["onnx/*", "*.gif", "*.ipynb"]` (constant `BACKBONE_SKIP`)
  to `snapshot_download`. Fresh prefix `~/dev/g1-ext/robot-day-test2`: `du -sh hf_cache/hub/blobs` = **2.7 GB** (was 7.5 GB), the
  same size as Finnegan's working cache; SmolVLM2 snapshot has no `onnx/`. `--release-file` -> `SETUP release: OK`
  (runs/real_e_setup_release_r2.log), `--policy-env` -> `SETUP policy-env: OK` (runs/real_e_setup_policy_r2.log), 81 `--self-test`
  on CPU from that prefix with `HF_HUB_CACHE=$P/hf_cache/hub` -> `SELF-TEST PASS` (runs/real_e_policy_selftest_r2.log,
  runs/real_e_policy_cache_r2.log exit 0). Numbers corrected: script header "about 3 GB on a Mac, about 6 GB on Ubuntu with the
  CUDA torch; 20-60 min"; runbook A3 "about 4 GB on a Mac, about 7 GB on Ubuntu with an NVIDIA card; 20-60 min" (adds the
  0.9 GB model). Basis: measured 2.7 GB backbones + 0.87 GB tar + 0.85 GB installed Mac venv. The Ubuntu CUDA torch size is
  inferred (not measured, there is no Linux machine here).
- **E1-2 fixed.** New tests: `test_loopback_pub_refuses_non_loopback_iface`, `test_bound_interfaces_reads_selected_line` (it uses
  the real trace line `selected interfaces: lo0 (index 1 priority 2)`, captured from /tmp/cdds.LOG after a 1 s helper run on lo0,
  runs/real_e_loopback_trace_r2.log), and `test_loopback_pub_refuses_when_dds_bound_another_iface` (main() with a trace file saying
  en0 -> exit 2, nothing published). Mutation checks M1 (no non-loopback refusal), M2 (no bound-interface refusal), M3 (parser
  broken) are all seen failing (runs/real_e1_mutation_r2.log).
- **E1-3 fixed.** Runbook A5 has a new "if you see" line: `command not found: python` or `ModuleNotFoundError` (no CHECK line)
  -> run the source line again. `CHECK versions: FAIL` now says "wrong environment". 89 itself is unchanged: the optional import
  catch was not done.
- **E1-4 fixed.** Note under the Monday 08:00 block: on the laptop without the policy env, add `--skip-policy`.
- **E1-5 fixed.** `release_download` skips `gh release download` when the tar and its .sha256 already pass the checksum. Tested with
  a prefix whose release/ already held the tar: "already downloaded and checksum OK - not downloading again", `SETUP release: OK`
  (runs/real_e_setup_release_skip_r2.log).
- **E1-6 fixed (not testable here).** A4 now runs `socketfilterfw --getglobalstate`. If the firewall is enabled, it `--add`s and
  `--unblockapp`s both the venv link and its `readlink -f` target, because we do not know which one macOS checks. There is also a
  port-8765 line for the policy-server laptop (macOS: the same lines for policy-env; Ubuntu: `sudo ufw allow 8765/tcp`). The
  firewall on this Mac is disabled ("State = 0"), so whether macOS honours the link is still not run.
- **E1-7 fixed.** The test now uses a `FakeClock` class and a named `robot_net_now` answer list, so the old tricks are gone. The
  bash expansions have one-line comments. 89 uses `tuple[str, str]`, `last_line` has a docstring, the lambda became
  `skipped_policy()`, and `DRY_RUN_TIMEOUT_S` has a source comment. `make_robot_day_data.py` reads fps from meta/info.json
  (`dataset_fps`). The regenerated npz/png files are array-identical to the old ones. The flags test now also reads inline
  `...scripts/...` commands (mutation M5 seen failing). The new `test_runbook_84_steps_exist` checks the --steps values against 84's
  STEPS (mutation M4 seen failing).
- Re-run: unit tests 44 pass in the sim env (runs/real_e_unit_r2.log), 10 pass + 4 SDK skips in the project .venv
  (runs/real_e_unit_projectvenv_r2.log). Robot env into the fresh prefix: OK. **Full 89 from the fresh prefix, CPU policy:
  SELFTEST: PASS**, all 6 checks (runs/real_e_89_r2.log).
- Adjacent, not fixed (outside this round): 89 measured loopback LowState at **112 Hz**, but the helper is set to 200 Hz
  (macOS sleep jitter). 89 accepts it because the loopback threshold allows it. The helper comment "at least 84's robot refuse rate"
  is therefore not true on this Mac; worth a look at integration.

# Track E1 - round 2

Reviewer: independent code reviewer (did not write Track E1, saw no author reasoning). Finished 2026-10-18 18:08.
Reviewed against PLAN.md rev 6.3 (Track E1 task incl. the (R3-x) items "that arrive through E1's review and fix round", sections 0,
3.6, 6.2) and the round-1 findings E1-1..E1-7.

## What I ran (full output under runs/)
- Unit tests, sim env (`~/dev/g1-ext/.venv`): `pytest tests/test_unit_runbook.py tests/test_unit_policy_wire.py -v -rs` -> **44 passed**
  (runs/review_e1_r2_unit.log). Project .venv: 39 passed, 5 skipped with the reason "could not import unitree_sdk2py"
  (runs/review_e1_r2_unit_projectvenv.log).
- Own mutations on copies of tests/helpers/loopback_lowstate_pub.py (scratchpad, repo untouched), runs/review_e1_r2_mutation.log:
  M0 unmodified -> 5 pass; M1 robot-network refusal removed -> FAILS (caught); M2 mid-run re-check removed -> FAILS (caught);
  M3 bound-interface refusal removed -> FAILS (caught); M5 re-check only every 5 s -> FAILS (caught); M4 (double publish, a benign
  control mutation) passes, as expected. The three safety refusals of the helper are now all mutation-proven.
- 89's `check_policy` against a temp prefix with the real policy env and model but an EMPTY `hf_cache/hub` (no SmolVLM2 folder),
  CPU, no DDS: returns `('FAIL', '81 --self-test on cpu exit 1; full output in runs/robot_day_selftest_policy_cpu.log')`; the log
  ends in `LocalEntryNotFoundError` (runs/review_e1_r2_policy_nobackbone.log). (This overwrote runs/robot_day_selftest_policy_cpu.log.)
- Setup script under macOS /bin/bash 3.2.57: `bash -n` OK, `--show-ip` OK exit 0, unknown option -> usage, exit 2
  (runs/review_e1_r2_setup_static.log).
- Timing of the loopback helper's loop body in the robot env (no DDS): `CRC().Crc(LowState_)` = 1.33 ms, `time.sleep(0.005)` = 6.19 ms
  on this M4.
- Read: all round-1 fixes in setup_robot_laptop.sh, 89, loopback helper, make_robot_day_data.py, runbook, tests.

## Not run (named, not passed)
- Full 89 with the DDS check: I started no DDS (section 0 / R3-4). The author's run (runs/real_e_89_r2.log) shows SELFTEST: PASS with
  `LowState 112 Hz`; the push-1 check (6.2) runs it again.
- Ubuntu x86_64 path (uv `--torch-backend auto`, gh from apt, CycloneDDS on Linux `lo`): no Linux machine.
- macOS firewall enabled path (A4): the firewall on this Mac is off.

## Round-1 findings: status
- E1-1 (major) fixed: `BACKBONE_SKIP` passed as `ignore_patterns`; author's fresh prefix 2.7 GB blobs + CPU self-test PASS; sizes
  corrected in header and A3.
- E1-2 fixed: three new helper tests; my M1/M2/M3 mutations are now caught.
- E1-3, E1-4 fixed in the runbook (A5 hints; --skip-policy note under the 08:00 block).
- E1-5 fixed: `release_download` skips when the local tar passes its checksum.
- E1-6 fixed in text (getglobalstate, link + readlink target, port 8765); still not exercised (firewall off here).
- E1-7 mostly fixed (FakeClock, bash comments, tuple types, fps from info.json, inline-command + --steps tests). Leftover below (E1-12).

## Verdict 1 - did the code do what the plan said?
No, one planned item is missing (E1-8). Everything else in the E1 task and the round-1 fixes is present and works as described.

## Verdict 2 - is the plan right for real hardware / the operators' laptops?
Mostly. One gap the plan does not cover: the loopback rate margin (E1-9). The read-only safety design is sound: guard installed
before any other import in 89, DDS refused while 192.168.123.x exists, the only publisher is a bounded separate process with three
mutation-proven refusals (non-loopback iface, robot address at start and every second, bound interface).

## Findings

**E1-8 (major) - R3-1's 89 part is not built: no backbone-folder check, PASS line does not name the cache, planned test missing.**
- Plan: 3.6 (89 `policy`: "FAIL when `<prefix>/hf_cache/hub/models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct` is missing
  ('backbone not in the prefix - re-run setup_robot_laptop.sh --policy-env'), and the PASS line names that cache path"), E1 step 3,
  E1 step 6 `test_selftest_policy_fails_without_backbone_in_prefix`, and 6.2 push-1 pass criterion "the `policy` line naming
  `$P/hf_cache/hub`".
- Where: scripts/89_robot_day_selftest.py:138-156 checks only `policy-env/bin/python` and the checkpoint dir; the PASS line (156) is
  `81 --self-test on cpu in 10 s (load_s ...)` (runs/real_e_89_r2.log) - no cache path. `grep SmolVLM2` in 89: nothing. No such test
  in tests/test_unit_runbook.py.
- Trigger / impact: an operator laptop whose `--policy-env` download broke after the venv was created (Wi-Fi drop, disk full) gets
  `CHECK policy: FAIL 81 --self-test on cpu exit 1; full output in ...` (reproduced with an empty cache). That matches the runbook's
  "any other FAIL -> send to Finnegan, go on with the other laptop" instead of A5's "`... missing` -> run A3 again" - a one-command
  fix turns into a lost laptop on Monday. And the push-1 check (6.2) cannot meet its own pass criterion, because the PASS line never
  names `$P/hf_cache/hub`. (Safety itself holds: HF_HUB_CACHE + HF_HUB_OFFLINE are passed, so Finnegan's old cache is not used.)
- Fix: in `check_policy`, a named constant for the backbone folder; FAIL with "backbone not in the prefix - re-run
  setup_robot_laptop.sh --policy-env" when it is missing (before starting 81); add `cache <prefix>/hf_cache/hub` to the PASS detail;
  add `test_selftest_policy_fails_without_backbone_in_prefix` (temp prefix, stub policy-env python + checkpoint dir, no backbone ->
  FAIL naming the setup command; with Proves/Catches) and show it failing on a copy without the check.

**E1-9 (major) - the loopback helper reaches only ~112 Hz against a 100 Hz refuse limit; slower laptops will fail 89 and 84 falsely.**
- Where: tests/helpers/loopback_lowstate_pub.py:63-73 sleeps a fixed `1/hz` AFTER the work, so the period = sleep jitter + CRC +
  write. Measured here: sleep(5 ms) = 6.19 ms, CRC = 1.33 ms -> about 8.9 ms = 112 Hz, the same as 89 measured (runs/real_e_89_r2.log).
  The limit is `LOWSTATE_HZ["loopback"]["refuse"] = 100` (g1sim/real_io.py:51), used by 89 (line 87) and by 84 in the push-1 check.
- Trigger: the operators' laptops (unknown, likely older/slower than an M4; CRC is pure Python): CRC 2x slower already gives ~100 Hz.
  This is inferred from the measured numbers, not reproduced on a slow laptop.
- Impact: `CHECK dds: FAIL LowState at 9x Hz ... laptop overloaded?` on a healthy laptop; per the runbook the operators then drop
  that laptop. The helper comment "at least 84's robot refuse rate ... so the self-test is not marginal" (line 30) is false today.
- Fix: schedule against a deadline (`next_tick += 1/hz; sleep(max(0, next_tick - clock()))`) so work time and jitter do not add up;
  one unit test with the FakeClock that a slow `publish_lowstate` (e.g. advancing the clock 2 ms) still yields ~hz publishes per
  second. Re-run 89 at the push-1 check and record the measured rate.

**E1-10 (minor) - runbook B4 drops the plan's distance.** Plan E1 step 5: "both operators stand outside the arms' reach (1.5 m)";
runbook line 190 says only "outside arm reach". Fix: "at least 1.5 m from the arms".

**E1-11 (minor) - long copy-paste commands have no anti-sleep prefix.** Page 1 rule 4 tells operators to put `caffeinate -i` /
`systemd-inhibit` in front of long commands, but rule 1 says run commands "exactly as written", and A3 (20-60 min), the 81 server
and B4 are written without it. Fix: write the long commands with the macOS prefix and give the Ubuntu line next to them.

**E1-12 (minor) - clean-code leftovers.** 89:186 still builds the policy entry with a conditional lambda inside the dict (round 1
asked for a plain function); write `def policy_check(): ...` or an if/else before the dict. loopback_lowstate_pub.py:47 `-> list` and
:105 `Optional[list]` should be `list[str]`.

**E1-13 (minor) - A4 policy-server firewall line needs editing, and `readlink -f` needs macOS 12.3+.** "the same four lines with
`policy-env` instead of `robot-env`" makes the operator edit a command (rule 1). Fix: write the policy-env lines out; note that
`readlink -f` exists from macOS 12.3 on (or use `python -c "import os,sys;print(os.path.realpath(sys.argv[1]))"`).

## Gate
Track E1 - round 2: **NO-GO** (2 majors: E1-8 R3-1 backbone check + planned test missing in 89; E1-9 loopback rate margin 112 vs
100 Hz. 4 minors). Both majors are small: about 30-45 min including tests and mutation checks. Full 89 DDS run, the Ubuntu path
and the firewall-enabled path stay "not run".

## Track E1 - response (fix round 2, implementer, 2026-10-18 18:11)

All 2 majors and 4 minors fixed; nothing refuted.

- **Major 1 (89 backbone, R3-1):** `scripts/89_robot_day_selftest.py` has `HF_CACHE` and `BACKBONE_DIR` constants; `check_policy` FAILs
  before starting 81 with "backbone not in the prefix (<path> missing) - re-run setup_robot_laptop.sh --policy-env", and the PASS
  detail ends with "cache <prefix>/hf_cache/hub". New test `test_selftest_policy_fails_without_backbone_in_prefix` (fake 81 returns
  exit 1, log goes to tmp_path). Mutation M-R2-1 (check removed): test FAILS with exactly the reviewer's symptom
  "81 --self-test on cpu exit 1" - runs/real_e1_r2_mutation.log. Runbook A5 has a matching "if you see" line.
- **Major 2 (loopback rate):** `run_loop` schedules against a deadline (`next_tick += 1/hz; sleep(max(0, next_tick - clock()))`);
  the "not marginal" comment is replaced. New test `test_loopback_pub_keeps_rate_when_publish_is_slow` (FakeClock, 1.5 ms publish,
  1.2 ms sleep overshoot -> >= 190 Hz). Mutation M-R2-2 (old fixed sleep): test FAILS with 130 Hz - runs/real_e1_r2_mutation.log.
  Real-clock measurement without DDS (real CRC, no writer; section 0 forbids loopback DDS in track builds), robot env:
  200.0 Hz for 5 s - runs/real_e_rate_nodds.log. The rate with the real DDS write stays for the main session's push-1 check.
- **Minor B4 1.5 m:** "at least 1.5 m from the arms".
- **Minor caffeinate vs rule 1:** rule 4 now says long commands are printed twice (macOS `caffeinate -i` line, Ubuntu
  `systemd-inhibit` line) and to run the one for your laptop; A3, A5, the Monday 08:00 89 run, B4 and the 81 server are printed both ways
  (81: mps on macOS, cuda on Ubuntu).
- **Minor clean code:** the conditional lambda is a small `policy_check` function; `bound_interfaces -> list[str]`,
  `main(argv: Optional[list[str]])`.
- **Minor A4:** the policy-env firewall lines are written out in full; `readlink -f` replaced by the venv's own Python
  (`os.path.realpath(sys.executable)`), which works on every macOS version.

Verify: `$PY -m pytest tests/test_unit_runbook.py tests/test_unit_policy_wire.py -v -rs` -> 46 passed, 0 skipped (86 now exists) -
runs/real_e_unit.log.

## Track E1 - round 3

Reviewer: independent code reviewer (did not write Track E1, saw no author reasoning). Finished 2026-10-18 18:13.
Reviewed against PLAN.md rev 6.3 (sections 3.6, 4.1, Track E1 task) and the round-2 findings E1-8..E1-13.

### What I ran (full output under runs/)
- Unit tests, sim env: `~/dev/g1-ext/.venv/bin/python -m pytest tests/test_unit_runbook.py tests/test_unit_policy_wire.py -v -rs`
  -> **46 passed, 0 skipped**, exit 0 (runs/review_e1_r3_unit.log).
- Own mutations on scratch copies (repo untouched), runs/review_e1_r3_mutation.log:
  - M1 89 backbone-folder check removed -> **FAILS** (caught; message "81 --self-test on cpu exit 1", started 81).
  - M3 loopback helper back to a fixed `sleep(1/hz)` -> **FAILS** (caught by test_loopback_pub_keeps_rate_when_publish_is_slow).
  - M4 robot-network re-check only every 5 s -> **FAILS** (caught).
  - M2 89 no longer passes `HF_HUB_CACHE` to 81 -> **survives** (E1-14).
  - M5 89 PASS line no longer names the cache -> **survives** (E1-14; push-1 check reads this line by eye).
- Read: scripts/89_robot_day_selftest.py (213 lines), tests/helpers/loopback_lowstate_pub.py (141), tests/test_unit_runbook.py (272),
  runbook A4/A5/B4/81 blocks.

### Not run (named, not passed)
- Full 89 with real loopback DDS (the helper's rate with the real DDS write): section 0 forbids DDS in track builds; push-1 check (6.2).
- Ubuntu path, macOS firewall-enabled path (firewall off here), fresh-prefix policy self-test (author's earlier run, not repeated).

### Round-2 findings: status
- E1-8 (major) fixed: `HF_CACHE` / `BACKBONE_DIR` named constants with sources (89:54-55); FAIL before 81 starts with
  "backbone not in the prefix (...) - re-run setup_robot_laptop.sh --policy-env" (89:146-147); PASS detail ends "cache <prefix>/hf_cache/hub"
  (89:160); planned test present with Proves/Catches; my M1 confirms it can fail. Runbook A5 has the matching "if you see" line.
- E1-9 (major) fixed: deadline scheduling in run_loop (loopback_lowstate_pub.py:77-78); FakeClock test with slow publish + sleep
  overshoot; my M3 confirms it fails on the old code. Average rate can never exceed hz (catch-up only fills missed ticks, loopback only),
  so the change cannot make an overloaded laptop pass falsely.
- E1-10..E1-13 (minors) fixed: "at least 1.5 m from the arms" (B4), long commands printed with both anti-sleep prefixes, policy_check
  is a plain function with a docstring, `list[str]` types, A4 policy-env firewall lines written out, realpath via the venv's Python.

### Verdict 1 - did the code do what the plan said?
Yes. Every E1 item of rev 6.3 (incl. R3-1 in 89 and its planned test) is present; the safety refusals of the only test publisher
(non-loopback iface, robot address at start and each second, bound interface) and the publisher guard first in 89 are intact and
mutation-proven. Clean code: functions are short, single-job, named constants carry sources, every test has Proves/Catches.

### Verdict 2 - is the plan right for real hardware / the operators' laptops?
Yes for push 1. The read-only design is sound. The real-DDS rate of the helper stays to be measured at the push-1 check (6.2).

### Findings

**E1-14 (minor) - no test proves 89 hands the prefix's own cache to 81; the new test's "Catches" overclaims.**
- Where: scripts/89_robot_day_selftest.py:148 (`env = dict(os.environ, HF_HUB_CACHE=str(cache), HF_HUB_OFFLINE="1")`) and :160.
  Mutation M2 (drop HF_HUB_CACHE) and M5 (drop "cache ..." from the PASS line) both pass all 46 tests.
- Trigger / impact: a later edit that drops the env entry makes 81 read the default HF cache. On Finnegan's Mac that holds SmolVLM2,
  so the push-1 self-test can PASS from his old cache - exactly what the test docstring says it catches. On the operators' laptops it
  fails loudly (default cache empty), so this is fail-safe, not a Monday safety risk; the current code is correct.
- Fix: one positive test - backbone folder present, fake `subprocess.run` that records `kwargs["env"]` and writes "SELF-TEST PASS"
  to the log - asserting `env["HF_HUB_CACHE"] == str(prefix / HF_CACHE)`, `HF_HUB_OFFLINE == "1"` and that the PASS detail names
  that path; or reword the existing "Catches" to what it proves (missing backbone -> setup hint, 81 not started).

Note (no finding): runbook A5's later line "`CHECK policy: FAIL ... missing`" also matches the backbone message; it comes after the
specific line and its remedy (re-run A3 with --policy-env) also fixes it, so it is harmless.

### Gate
Track E1 - round 3: **GO** (0 blockers, 0 majors, 1 minor E1-14 that may be fixed in E2 or later). The real-DDS loopback rate,
the Ubuntu path and the firewall-enabled path stay "not run" and belong to the push-1 check / Monday.
