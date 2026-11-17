# Runbook dry walk Parts A+B (read-only, before push 1)

Reviewer: "Cristina" (I am an operator who did not write the code). Date: 2026-10-18. I did not author the runbook or the scripts and
have not seen the author's reasoning.
File walked: `docs/ROBOT_DAY_RUNBOOK.md` at HEAD `c0898cf` (the working tree copy is unchanged vs HEAD). Part C ignored as instructed.

## What I ran (read-only)

- `--help` of `scripts/89_robot_day_selftest.py`, `scripts/84_real_bringup.py`, `scripts/86_real_run.py` (in the test robot env
  `~/dev/g1-ext/robot-day-test/robot-env.sh`, because `~/g1-robot-day` does not exist on this Mac): all exit 0. The flags the runbook uses
  exist: 84 `--iface --steps --robot-pc --camera-device --seconds --label`; steps `net,probe,joints,questions,hand-state,camera,record`;
  89 `--skip-policy`; 86 `gate`.
- `bash scripts/setup_robot_laptop.sh --help` and `bash -n`: OK; flags `--robot-env --policy-env --release --show-ip` exist.
- Clean copy of the committed tree (`git archive HEAD` into the scratchpad): `89 --help` and `84 --help` exit 0 under `python -I`, so all
  imports they need are committed; `tests/helpers/loopback_lowstate_pub.py`, `fake_camera.py`, `tests/data/selftest_episode.npz`,
  `docs/robot_day/reference_head_frame.png`, `scripts/71_real_probe.py` are tracked. `scripts/86_real_run.py` is NOT in HEAD (untracked),
  so on Monday 08:00 `86 gate` gives "No such file" until push 2 - the runbook covers that case correctly.
- Read every print/exit line that the runbook quotes as expected output (setup `ok()/failed()`, 89 `CHECK`/`SELFTEST`, 84 `STEP`,
  `FACTORY ARMS`, real_bringup reasons, real_camera grabber errors, 81 self-test timing line, 86 `gate_mode`).

## Checks I could not run (keep in mind; they are outside a paper walk)

- Whether the GitHub release `robot-day-2026-10-18` exists and whether `acristinaa` / `saadsheikh-tech` can read the repo and the
  release (would need the network; forbidden here).
- The Ubuntu install path (CUDA torch via `--torch-backend auto`, `systemd-inhibit`, `ufw`): only the macOS path has run (STATUS 17:51).
- The policy check on `mps` (89's default on an Apple-silicon Mac): every recorded 89 run used `cpu`
  (`runs/robot_day_selftest_20261018_*.json`, `runs/push1_selftest.log`).
- macOS `socketfilterfw` lines, and everything against the real robot.

## Verified as correct (no finding)

- A3 expected lines and their order: the script prints `SETUP robot-env: OK <python version, source ...>`, then `SETUP release: OK ...`,
  then `SETUP policy-env: OK ...` (setup_robot_laptop.sh:170-172), failures as `SETUP <block>: FAILED <hint>`. Matches the runbook (the
  OK lines carry a trailing detail, which is harmless).
- A5 FAIL texts `CHECK dds: FAIL a 192.168.123.x address exists`, `CHECK versions: FAIL`, `CHECK policy: FAIL backbone not in the prefix`,
  `CHECK policy: FAIL ... missing`, `SELFTEST: PASS` all match 89:65, 72, 144-147, 208.
- A6: `chunk_ms` is really in the `CHECK policy` PASS line (89:157,160 takes 81's `load_s ... chunk_ms ...` line, 81:138).
- Monday: `86 gate` prints exactly `MOTION: LOCKED` / `MOTION: UNLOCKED` (86:133).
- B0: `STEP net/probe/joints: PASS` and a `FACTORY ARMS: ...` line are printed (84:122, 261); the 200 Hz abort threshold equals
  `LOWSTATE_HZ["robot"]["refuse"] = 200` (real_io.py:51); `rt/arm_sdk` samples give FAIL (real_bringup.py:97-99).
- B1: 84 does have a `questions` step with Q1-Q12 and FAILs if Q8/Q10 stay empty (real_bringup.py:160-171).
- B3: "camera busy" does appear in the grabber error (real_camera.py:81).

## Findings

### F1 (major) B3/B4: there is no way to get `N` (the D455's /dev/videoN)
- Step: B3 "`N` = the D455's `/dev/videoN` number from Q2", reused in B4.
- What I'd do as Cristina: look at my Q2 answer. But Q2 as 84 asks it is "which computer is it plugged into, USB 3 or 2, and is another
  program using it?" (real_bringup.py:34) and the runbook's paper Q2 is "D455 host + USB port + is it already opened" - neither asks for
  the video number. The owner may not know it (a D455 exposes several /dev/video nodes: depth, IR, colour).
- What goes wrong: I cannot fill in `N`, so B3 and B4 (the "real data" that makes Part B a complete day) stop. If I guess wrong, 84
  prints `STEP camera: FAIL no frames (no frame from /dev/videoN (wrong number or camera busy)): check --camera-device with 72 kamera
  --suchen ...` - and `72` is a command the runbook forbids ("Anything that is not in this runbook"). The runbook's "if you see camera
  busy" line only covers the busy case, not "wrong number".
- Suggested fix: either allow and print the lookup command (e.g. `python scripts/72_real_check.py kamera --suchen ...` with its exact
  flags, stating it is read-only), or give the procedure "try N = 4, then 0..5 in order; the right one gives STEP camera: PASS", and add
  "and which /dev/videoN is the colour stream" to Q2 in both the paper list and real_bringup.QUESTIONS.

### F2 (major) B3/B4: the SSH login to the robot PC is never mentioned
- Step: B3 and B4 pass `--robot-pc unitree@192.168.123.164`; 84 opens `ssh unitree@192.168.123.164 python3 - ...` (real_camera.py:107-114).
- What I'd do: run the command and wait for an image.
- What goes wrong: on a fresh laptop ssh first asks "Are you sure you want to continue connecting (yes/no/[fingerprint])?" and then
  "unitree@192.168.123.164's password:". The runbook says neither, gives no password (Q5 as asked by 84 is only "user and IP"), and 84
  gives up after 30 s (`INFO_TIMEOUT_S`, real_camera.py:29) with `camera did not start (grabber ended before its info line) ...`. In B4 the
  password prompt comes again on every recording. An operator who has never seen this will think the step is broken.
- Suggested fix in B3: "84 opens an SSH connection to the robot PC. The first time it asks `continue connecting (yes/no...)?` - type
  `yes`. Then it asks for the password - ask the owner (Q5; Unitree's default is often `123`) and type it within 30 s. B4 asks again on
  every recording." Add "and the password" to Q5.

### F3 (major) B3: "nominal position" of the table is undefined, and "adjust the table/camera" invites changing the camera tilt
- Step: B3 bullets 2-3 and "Adjust the table/camera until the overlay lines up".
- What I'd do: I have no number for "the nominal position" (how far from the robot, where the volleyball stand goes). The plan has it
  (PLAN.md:203-204: pelvis 0.2455 m behind the table's front edge at the walk stop, ball at (0.344, 0.241) on a tube stand) but the
  runbook does not. Then, seeing a misaligned overlay, I would tilt the camera as well, because the text says "table/camera".
- What goes wrong: the tape marks Part C relies on are placed by guess, the stand/ball may be missing from the scene, and the camera can
  end up away from the 50 deg it was just set to - silently breaking the very match with the training picture the step exists for.
- Suggested fix: give the placement in metres (table front edge 0.25 m in front of the hip joint axis, centred; stand + volleyball at the
  position used in training, with a photo or `docs/robot_day/reference_head_frame.png` named as the picture to copy), and write "keep the
  D455 at 50 deg; move only the table (and stand) until table edge and stand line up in the overlay".

### F4 (major, conditional - Ubuntu 22.04 laptops) A1: `sudo apt install gh` gives a gh too old for the setup script
- Step: A1 "Ubuntu: `sudo apt install git gh curl`", then A3 `--release`.
- What I'd do: run the apt line, it works, `gh auth login` works.
- What goes wrong (inferred from the gh changelog as I remember it, NOT verified here - no network): Ubuntu 22.04's archive ships gh
  2.4.0; the script calls `gh release download ... --clobber` (setup_robot_laptop.sh:154), a flag added in a later gh release. The block
  then prints `SETUP release: FAILED download of robot-day-2026-10-18 failed - gh auth status ok? internet? (865 MB)`, which points me at
  the wrong cause; re-running fails the same way and that laptop never gets the model. Ubuntu 20.04 has no `gh` package at all.
- Suggested fix: after the apt line add "check `gh --version`: it must be 2.20 or newer; if older or 'Unable to locate package gh',
  install gh from https://cli.github.com (the 'Installing gh on Linux' apt instructions)". Someone with network should confirm the
  version that introduced `--clobber` before push 1.

### F5 (major) A3/A6: "the robot laptop" is never defined, so A3's "may skip the policy env" cannot be decided
- Step: A3 "A laptop that will **not** serve the model may skip the policy env", A6 "On an Apple-silicon Mac the robot laptop runs it
  itself. Otherwise use the laptop whose `CHECK policy` line passed...".
- What I'd do: on Sunday I do not know which laptop is "the robot laptop" (the plan defines it as the one cabled to the robot, PLAN.md:123;
  the runbook never says so), and A6 - the decision - comes after A3/A5, and needs a passing policy check, i.e. the policy env installed.
- What goes wrong: Saad or I skip the policy env on the laptop that will need it (e.g. the Apple-silicon Mac that ends up on the cable),
  and Monday morning loses 20-60 min of downloads on venue Wi-Fi before Part C, or both skip it and A6 has no candidate.
- Suggested fix: Page 1: "Robot laptop = the laptop that gets the robot cable in Part B; decide it now and write it on the sheet." A3:
  "Install the policy env on every Apple-silicon Mac and every Ubuntu laptop with an NVIDIA card. Skip it only on other laptops (Intel
  Mac: torch 2.11 has no Intel-Mac build; Ubuntu without NVIDIA: CPU is too slow)."

### F6 (minor) A6: `chunk_ms` is a cold first-call time, and there is no instruction when it is >= 1500 ms
- 81's self-test times the very first inference (81:129-131), which includes one-time warm-up on mps/cuda; the warm MPS figure in
  PROGRESS.md:122 is 207 ms. A6 also does not say whether the 1500 ms rule applies to the Apple-silicon robot laptop, nor what to do when
  no laptop is below 1500. Fix: "If chunk_ms >= 1500, run 89 once more (the second load is warm); still >= 1500 on every laptop: write it
  down, Part C runs without the VLA steps" (or whatever the plan wants), and say the rule applies to every candidate.

### F7 (minor) B0 `<IF>`: `--show-ip` prints the name with a trailing colon
- `setup_robot_laptop.sh --show-ip` prints `  en5: 192.168.123.222` on macOS (ifconfig `$1` keeps the colon, line 162) and
  `  enx...: 192.168.123.222/24` on Ubuntu (line 164). If I copy `en5:` into `--iface`, 84 says `STEP net: FAIL en5: has no IPv4 address:
  set 192.168.123.222/24 ...`, which looks like the IP was not set. Fix: "`<IF>` = the name in front of the colon, without the colon".

### F8 (minor) A5: "with the A3 prefix" is ambiguous
- Line 120-121: "run `bash scripts/setup_robot_laptop.sh --policy-env` again (with the A3 prefix)". The FAIL line itself talks about the
  install folder as "the prefix" ("backbone not in the prefix"), and setup has a `--prefix` flag. I could read it as "add `--prefix`".
  Fix: "with `caffeinate -i` (macOS) or `systemd-inhibit --what=sleep:idle:handle-lid-switch` (Ubuntu) in front, as in A3".

### F9 (minor) A5: expected output with `--skip-policy`, and where the policy failure detail is
- With `--skip-policy` the policy line is `CHECK policy: SKIP --skip-policy` (89:165,200), not PASS; the runbook says "one `CHECK <name>:
  PASS` line per check". A policy FAIL from 81 itself reads `CHECK policy: FAIL 81 --self-test on mps exit 1; full output in
  runs/robot_day_selftest_policy_mps.log`; "any other FAIL" asks only for the json. Fix: "(on a --skip-policy laptop the policy line says
  SKIP - that is fine)" and "send the json and, for a policy FAIL, the `runs/robot_day_selftest_policy_*.log` it names".

### F10 (minor) A4: forward reference "(A6)"
- A4 asks the laptop that serves the policy to open port 8765, but that decision is made in A6. Fix: "If you do not know yet which laptop
  serves the policy, do this on every laptop that installed the policy env."

### F11 (minor) B0: what to do when ping fails but the cable is right
- B0 says "three ping replies. If not: check the cable and the address once, then ask the owner which port to use" - and then? 84 treats
  a silent PC2 as a warning only (real_bringup.py:81-83, "the probe step decides"). Fix: "Then run the 84 line anyway; its probe step
  decides."

### F12 (minor) B0: 84's own FAIL advice vs the runbook's abort list
- For `< 200 Hz` 84 says "close other programs, re-run" (real_bringup.py:95), for stale LowState "check cable, IP and the robot is on";
  the runbook says "Abort Part B". Fix: "do what the FAIL line says once (rule 3); still FAIL: abort Part B". Also say that `STEP <x>: NOT
  RUN not selected (--steps)` lines for unselected steps and the final `OWN WRITERS: 0 (must be 0)` line are normal.

### F13 (minor) B2: "six values per hand" are not printed
- The PASS line is `STEP hand-state: PASS both hands report (0 = open, 1 = closed)` (real_bringup.py:195); the six values are only in the
  `runs/real_bringup_*.json` report. Fix: "You should see `STEP hand-state: PASS both hands report ...`".

### F14 (minor) B3: where the overlay image is and what "lines up" means
- 84 does not show an image; it prints `  saved runs/real_camera_overlay_<stamp>.png` (84:165). Fix: "open the file named in the `saved
  runs/real_camera_overlay_...png` line (macOS `open <file>`, Ubuntu `xdg-open <file>`); it is a 50/50 blend with the training picture:
  the table front edge and the stand must sit on top of each other."

### F15 (minor) B3/B4: `--robot-pc` is hard-coded
- Q2/Q5 may say the D455 is on another computer (e.g. the backpack PC, Q1) or another user/IP. Fix: "`unitree@192.168.123.164` is the
  default; use the user@IP from Q5 if the owner gave another one".

### F16 (minor) B4: no expected output
- Fix: "You should see `STEP record: PASS 600 frames -> runs/real_observe/<stamp>_layout1.npz`" (84:195; 20 s x 30 fps). Also say what
  a "table layout" is (ball/stand positions?), and that a FAIL with "foreign rt/arm_sdk samples" means: stop, ask the owner.

### F17 (minor) FAIL hints in the scripts point to commands the runbook forbids
- 84 hand-state FAIL: "72 hand --pruefen shows whether it runs" (real_bringup.py:190); camera FAIL: "72 kamera --suchen"
  (real_bringup.py:203). Page 1 says never run anything not in the runbook. Fix: add one line to Page 1 "If a FAIL line suggests a `72 ...`
  command, do not run it unless this runbook lists it" (or list the read-only ones; see F1).

### F18 (minor) B1 wording drift between the paper list and 84
- Paper Q9 "robot clock" vs 84 Q9 "Robot time: how long may the robot run today, and when are the breaks?" (real_bringup.py:41) -
  "robot clock" reads like the system time. Q4/B2 say `brainco_hand_service`, the old `72` docstring says `brainco_hand_server`. Fix: copy
  the 84 wording into the paper list.

### F19 (minor) Undefined props: "the paper sheet", "the shared drive"
- A6/B0/B3 write to "the paper sheet"; end of day copies `runs/` "to the shared drive". Neither is defined anywhere in the repo. Fix: "bring
  a blank sheet; fields: robot laptop, R11 laptop + chunk_ms, FACTORY ARMS line, pelvis height, table height, Q8, Q10" and name the drive
  (link/folder).

### F20 (minor) Part B assumes one terminal
- Only B0 sources the robot env; B1-B4 rely on the same terminal. Fix: "Do all of Part B in this one terminal; if you open a new one, run
  `cd ~/UnitreeG1-Robot` and `source ~/g1-robot-day/robot-env.sh` first".

## Verdict

Every command and flag in Parts A+B exists exactly as written and every quoted output string that exists in the code matches. But four
majors would stop or mislead an operator who did not write the code: F1 (no way to find the camera number), F2 (an SSH password prompt the
runbook never mentions, with a 30 s timeout), F3 (table "nominal position" undefined plus an invitation to re-tilt the camera), F5 ("robot
laptop" undefined so the policy-env decision in A3 cannot be made). F4 is major only for an Ubuntu 22.04/20.04 laptop and is
inferred, not verified. All are text-only fixes to the runbook (F1 may also add a line to real_bringup.QUESTIONS).

NO-GO - fix F1, F2, F3, F5 (and F4 if either operator laptop is Ubuntu) before push 1; the minors can be folded into the same edit.
Open regardless: release/repo access and the Ubuntu install path were not checked (network forbidden in this walk).

## Dry-walk A+B fixes

Fixer: implementer agent (opus), 2026-10-18. Only Page-1-free text of Parts A and B in `docs/ROBOT_DAY_RUNBOOK.md` was changed
(Part A intro, A1, A3, A4, A5, A6, Part B intro, B0-B4, end of day); Part C, Page 1 and the Monday 08:00 block are untouched.
Gate: `~/dev/g1-ext/.venv/bin/python -m pytest tests/test_unit_runbook.py -q` -> 18 passed, 1 skipped (87 report not written yet),
full output `runs/real_runbook_ab_fix.log`; the new `72_real_check.py kamera --suchen` line is now flag-checked by
`test_runbook_commands_match_script_flags` (7 scripts, all pass).

- **F1 (major) camera number N:** B1 now has the operator add "which /dev/videoN is the colour stream" to the Q2 answer (paper Q2
  says the same). B3 adds the read-only lookup `python scripts/72_real_check.py kamera --suchen` (lsusb + `v4l2-ctl --list-formats`
  per /dev/video* + opencv version over SSH, `scripts/72_real_check.py:152-155, 225-230`), how to read it (YUYV = colour, Z16 depth,
  GREY infrared, empty = data-only node; lowest YUYV first), what the first line (`8086` / `keine Intel-RealSense am Bordrechner`)
  and last line (`opencv ...`) mean, and a fallback "try N = 0..7, the right one gives STEP camera: PASS" (no v4l2-ctl, or another
  user@IP, since 72 hard-codes `unitree@192.168.123.164`). The FAIL text `wrong number or camera busy` (real_camera.py grabber) and
  `it must run at 1280 x 800` (real_bringup.py check_camera) now each have an action.
- **F2 (major) SSH login:** B3 has an on-site login step (Finnegan's fact via the coordinator: login details exist only at SAP, from
  the owner): ask the owner for user + password, confirm the IP, then log in once by hand with `ssh unitree@192.168.123.164 echo ok`
  - expected host-key prompt (type `yes`), password prompt (nothing shows while typing), output `ok`; `Permission denied` /
  `Connection timed out` / `No route to host` each have an action. Says that every 72/84 camera command asks again and must get the
  password within 30 s (`INFO_TIMEOUT_S`, real_camera.py:29), else `camera did not start ...`. No password is written anywhere; B1
  says never to type it into 84 (answers are saved in `runs/`) or onto the sheet. Part A does not depend on SSH (stated in its intro).
- **F3 (major) table position + camera re-tilt:** B3 now gives the training layout from PLAN.md 2.7b re-derived from
  `g1sim/table_scene.py` + `navigation.table_stop_pose`: robot faces the table (yaw 0), front edge 24.5 cm in front of the hip joint
  axis (pelvis x = -0.0455, edge x = 0.20; the hip pitch joint has x offset 0 from the pelvis in the MJCF), table centred
  left-right, 90 x 65 cm in training; ball on its stand 14.4 cm back from the edge and 24.1 cm left, basket centre 14.4 cm back and
  24.1 cm right. Overlay: open the `saved runs/real_camera_overlay_<time>.png` file, it is a 50/50 blend with
  `reference_head_frame.png`. **Deviation from the suggested fix:** the reviewer suggested "move only the table until it lines up".
  I wrote instead: re-measure 24.5 cm and 50 deg once (camera only within 49-51), then if still off do **not** move the table off its
  tape marks or tilt further; write down what is off and go on. Reason: Part C's M0 lookup (87) is computed for the nominal table
  position and uses the Part B tape mark as "nominal", so moving the table to match the picture would silently void the arm-vs-table
  clearance. Open decision for Finnegan: may the table move to match the picture, and by how much?
- **F4 (conditional major) Ubuntu gh:** checked on the web today: (1) GitHub's official Linux instructions
  (raw `cli/cli` `docs/install_linux.md`, fetched 2026-10-18): Debian/Ubuntu install via the cli.github.com apt source (block copied
  verbatim); (2) GitHub release notes via the API: `release download --clobber` was added in **v2.15.0** ("`run/release download`:
  add `--clobber` and `--skip-existing` options", PR 6053) and a clobber bug in `release download` was fixed in **v2.21.0** (PR 6694);
  current gh is v2.102.0 (2026-09-30); (3) packages.ubuntu.com: jammy (22.04) ships gh **2.4.0+dfsg1-2** (too old), noble (24.04)
  2.45.0. A1 now: Ubuntu `sudo apt install git curl` + GitHub's official apt block for gh, then `gh --version` must show 2.21 or
  newer (re-run the block if older; it upgrades). So the reviewer's inference was right; the step now works on 22.04 and 24.04.
- **F5 (major) "robot laptop" undefined:** Part A intro defines it (the laptop that gets the robot cable in Part B and runs 84/86)
  and has the operators decide it before A1 and write it on the sheet. A3 now says who installs the policy env: every Apple-silicon
  Mac and every Ubuntu laptop with an NVIDIA card; only others skip it (CPU too slow: chunk_ms 2646-6450 measured in
  `runs/l1w_s4_t1_selftest.log`, `runs/real_e_policy_selftest*.log`, `runs/push1_selftest.log`).
- **F6 (minor) cold chunk_ms:** A6 rewritten as three steps: candidates (R11 order: Apple-silicon robot laptop first, then the
  other laptop with an mps/cuda pass; a cpu pass does not count), chunk_ms < 1500 for every candidate incl. the robot laptop, with
  the note that it is the first (cold) answer (1318 ms cold on MPS `runs/l1w_s8_mps_selftest.log` vs 207 ms warm, PROGRESS.md:122)
  and one 89 re-run if >= 1500; none below 1500 -> sheet + tell Finnegan, no policy server, so no M5/M6 (follows from the existing
  1500 rule; no new rule invented).
- **F7 (minor) `en5:` colon:** B0 shows both `--show-ip` formats and says `<IF>` is the name in front of the colon, without it.
- **F8 (minor) "A3 prefix":** replaced by the two full lines (`caffeinate -i ...` / `systemd-inhibit ...`) in a code block.
- **F9 (minor) SKIP/INFO lines + policy log:** A5 says `CHECK policy: SKIP --skip-policy` and `CHECK motion: INFO ...` are fine; a
  `CHECK policy: FAIL 81 --self-test ...` also sends `runs/robot_day_selftest_policy_<device>.log`.
- **F10 (minor) A4 forward reference:** port 8765 is now opened on every laptop that installed the policy env.
- **F11 (minor) ping fails:** B0: address right but no ping reply -> run the 84 line anyway, its probe step decides.
- **F12 (minor) abort list vs 84 advice:** B0 now lists each FAIL text with its one retry action (rule 3) and aborts only if still
  there; joint out of range aborts at once. The Part B intro explains `NOT RUN not selected (--steps)`, `OWN WRITERS: 0 (must be
  0)` and `REPORT ...` lines, and what to do if OWN WRITERS is not 0 (stop, tell Finnegan).
- **F13 (minor) six values:** B2 quotes the real PASS line and says the six values are in the `runs/real_bringup_<time>.json` report.
- **F14 (minor) overlay file:** see F3 (file name, `open` / `xdg-open`, what "lines up" means).
- **F15 (minor) hard-coded --robot-pc:** B3: another user@IP from the owner replaces `unitree@192.168.123.164` in the ssh line and
  every `--robot-pc`, and goes on the sheet; B4 refers back to it.
- **F16 (minor) B4 expected output:** `STEP record: PASS 600 frames -> runs/real_observe/<time>_layout1.npz`; a layout is defined
  (ball on stand + basket placement; move the stand at most 10 cm and the basket at most 8 cm = the training randomization in
  `configs/randomization.json`: objects offset_m 0.1, stand forward/sideways 0.1, walk basket_offset_m 0.08; table stays on its
  marks; put things back to the B3 placement afterwards); `foreign rt/arm_sdk samples while recording` -> stop, ask the owner;
  camera/lowstate stopped -> one re-run.
- **F17 (minor) 72 hints:** Part B intro: do not run a `72 ...` command a FAIL line suggests unless the runbook lists it (only
  `72_real_check.py kamera --suchen`, B3). Put in the Part B intro instead of Page 1 because only 84's Part B steps print 72 hints
  (and Page 1 stays untouched while Track E2 edits the file).
- **F18 (minor) wording drift:** the paper list now carries 84's exact question texts (real_bringup.py QUESTIONS), plus the video
  number in Q2; B2 names both `brainco_hand_service` and its program `brainco_hand_server`.
- **F19 (minor) sheet + shared drive:** Part A intro defines the sheet and its fields. The shared drive is not named anywhere in the
  repo or plan, so end of day now says "the shared folder Finnegan names in his message; none named: leave runs/ on the robot laptop,
  delete nothing". Open: Finnegan names the folder in the push-1 message.
- **F20 (minor) one terminal:** Part B intro: all of Part B on the robot laptop in one terminal; a new terminal first runs `cd` +
  `source`.
- **F21 (new, found while fixing F3; major) table height formula measured to the wrong point:** the runbook said "Table top height
  = 0.74 m + (pelvis height - 0.7505 m). Measure the pelvis height (floor to the hip joint axis)". 0.7505 m is the height of the
  pelvis body origin (`GROOT_STANCE["pelvis_z"]`), but the hip pitch joint sits 0.1027 m below it (MJCF
  `~/dev/g1-ext/g1_brainco_sim/assets/scene_29dof_brainco_volleyball_walk.xml`: `left_hip_pitch_link pos="0 0.064452 -0.1027"`). An
  operator measuring to the hip joint as told would set the table about 10 cm too low. B3 now says "0.74 m + (hip height - 0.648 m),
  hip height = floor to the hip pitch joint axis" (0.7505 - 0.1027 = 0.648). PLAN.md section 7 still says "pelvis height" without
  saying which point - harmless there, but worth one word when the plan is next edited.

## Dry-walk A+B fixes - needs code

Not changed (scripts are out of this task's scope); exact proposals:

1. `g1sim/real_bringup.py` QUESTIONS, Q2 (F1) and Q5 (F2):
   - `"Q2 D455: which computer is it plugged into, USB 3 or 2, and is another program using it?"` ->
     `"Q2 D455: which computer is it plugged into, USB 3 or 2, is another program using it, and which /dev/videoN is its colour stream?"`
   - `"Q5 SSH/IP: user and IP of the robot PC with the camera and hands (default unitree@192.168.123.164)?"` ->
     `"Q5 SSH/IP: user and IP of the robot PC with the camera and hands (default unitree@192.168.123.164)? (never type the password here: answers are saved)"`
2. `g1sim/real_bringup.py` check_camera FAIL text (F1/F17): `"check --camera-device with 72 kamera --suchen and that no other
   program uses the D455 (Q2)"` -> `"check --camera-device with python scripts/72_real_check.py kamera --suchen (runbook B3) and
   that no other program uses the D455 (Q2)"`. And check_hands FAIL text: drop `"; 72 hand --pruefen shows whether it runs"` (not
   in the runbook; the owner checks the service).
3. `scripts/72_real_check.py` kamera (F15): add `p.add_argument("--robot-pc", default=ROBOT_PC)` to the `kamera` sub-parser and
   pass it to `ssh(...)` (today `ROBOT_PC` is hard-coded, so the lookup cannot reach a camera on another user@IP); then the runbook
   line becomes `python scripts/72_real_check.py kamera --suchen --robot-pc <user@IP>`.
4. `scripts/setup_robot_laptop.sh` release_download (F4): before `gh release download`, check the version and fail with the real
   cause, e.g.
   `ver=$(gh --version | awk 'NR==1{print $3}'); minor=${ver#2.}; minor=${minor%%.*}; [ "${ver%%.*}" -gt 2 ] || [ "$minor" -ge 21 ] || { failed release "gh $ver is too old (needs 2.21+ for --clobber) - install gh from cli.github.com (runbook A1)"; return; }`
5. `real_camera.py` / 84 (F2, optional): a `camera did not start` message that names the likely cause when ssh's stderr contains
   `Permission denied` ("SSH login failed: password wrong or typed after 30 s").

## Review of dry-walk A+B fixes

Reviewer: independent reviewer agent (opus), 2026-10-18 ~18:35-19:00. I did not write the runbook, the fixes or the scripts and have
not seen the fixer's reasoning beyond the "Dry-walk A+B fixes" list above. Scope: the A+B hunks of `git diff -- docs/ROBOT_DAY_RUNBOOK.md`
against HEAD `c0898cf` (the Part C hunks in the same diff are not reviewed here), plus a coordinator add-on (Finnegan, mid-review):
the D455 sits on a motorized 2-DoF head module that the robot itself can move.

### What I re-derived / ran
- **F21 formula (re-derived):** MJCF `~/dev/g1-ext/g1_brainco_sim/assets/scene_29dof_brainco_volleyball_walk.xml:159,214`:
  `left/right_hip_pitch_link pos="0 +-0.064452 -0.1027"` in the pelvis frame (hip pitch joint at the body origin). Stance
  `g1sim/table_scene.py:69-76`: pelvis z 0.7505, pitch -0.96 deg. Hip pitch axis height = 0.7505 - 0.1027*cos(0.96 deg) = **0.6478 m**;
  its x shift from the pelvis origin is 1.7 mm (negligible). Table: `table` body at z 0.37, half height 0.37 (xml:615-616) -> top
  **0.74 m**, 0.0922 m above the hip axis. So `table top = 0.74 + (hip height - 0.648)` is **correct**, and the old text (0.7505 with
  "floor to the hip joint axis") would indeed have set the table ~10 cm too low. The dataset was recorded in `volleyball_walk`
  (PROGRESS.md:972, 997), i.e. at this stance - the right reference.
- **F3 numbers (re-derived):** `navigation.table_stop_pose((0.344, 0.241), (0.344, -0.241), 0.458)`: D = sqrt(0.458^2 - 0.241^2)
  = 0.3895, pelvis x = -0.0455, yaw 0; front edge x = 0.525 - 0.325 = 0.20 -> **0.2455 m**. Ball/stand centre 0.344 - 0.20 = **0.144 m**
  back, 0.241 left; basket centre 0.144 back, 0.241 right; table 0.65 x 0.90 m (xml:616). All match B3 and PLAN.md 2.7b.
  "Keep the table on the tape marks, camera only 49-51 deg" is consistent with Part C: the M0 lookup says "at the Part B tape mark,
  moved back N cm", i.e. 87 is computed relative to the nominal mark; camera tilt does not enter 87 at all.
- **F1 72 kamera --suchen:** `scripts/72_real_check.py` (tracked, unchanged): `kamera` sub-parser has `--suchen` (store_true) and
  `--geraet`; with no `--geraet`, `cmd_kamera` only runs `ssh(CAMERA_FIND)` = `lsusb | grep 8086`, `v4l2-ctl -d $d --list-formats`
  per /dev/video*, `python3 -c 'import cv2; print(opencv ...)'`, then prints a German hint. No DDS import, no ChannelFactoryInitialize,
  no publisher, nothing written. Not in the "NEVER" box (that lists `72 ... arm` and `72 ... hand --bewegen` only). Read-only: confirmed.
  Formats grepped are `YUYV|MJPG|Z16|GREY`; YUYV = D455 colour, Z16 = depth, GREY = IR (Y8), empty = metadata node: plausible (not
  checkable here without the D455).
- **F2 SSH:** no password value in any added line (grep of the added hunks: only "password" as a word and the IP). 30 s =
  `real_camera.INFO_TIMEOUT_S = 30.0` (real_camera.py:29, used at :144 before `camera did not start (...)`), consistent. Note: the
  pre-existing 72 docstring (line 9, not added text) says "Passwort laut LeRobot-Doku 123" - info only.
- **F4 gh (verified online today):** `gh api repos/cli/cli/releases/tags/v2.15.0`: "`run/release download`: add `--clobber` and
  `--skip-existing` options" (PR 6053); v2.21.0: "Fix clobbering existing files in `release download`" (PR 6694). So ">= 2.21" is
  correct (conservative). The apt block in A1 is byte-identical to `cli/cli` trunk `docs/install_linux.md:46-54` (fetched today).
  `setup_robot_laptop.sh:154` is the only gh feature beyond `auth`/`repo clone`.
- **F5:** robot laptop defined in the Part A intro; A3 policy-env rule (Apple silicon or NVIDIA) matches 89's
  `default_policy_device()` (89:136-140: mps / cuda if `nvidia-smi` / cpu) and setup's platform list (macOS arm64/x86_64, Linux x86_64).
- **Quoted strings:** `no fresh rt/lowstate`, `rt/lowstate only ... Hz`, `samples on rt/arm_sdk`, `outside the joint limits`
  (real_bringup.py:91-121), `OWN WRITERS: ... (must be 0)`, `REPORT` (84:245,269), `framing OK; check the overlay by eye`,
  `it must run at 1280 x 800` (real_bringup.py:206,216), `wrong number or camera busy` (real_camera.py grabber), `CHECK motion: INFO`
  (89:170), `STEP record: PASS <n> frames -> ...` at RECORD_FPS 30 (real_bringup.py:25, 84:198): all exist as quoted.
- **Gate:** `~/dev/g1-ext/.venv/bin/python -m pytest tests/test_unit_runbook.py -q` -> **18 passed, 1 skipped** (87's
  arm_path_clearance.json not written yet), exit 0, full output `runs/real_runbook_ab_review.log`.

### Implementation fidelity (did the fixes do what the list says)
Yes. Every item F1-F21 in "Dry-walk A+B fixes" is present in the A+B text as described; the one deviation (F3: do not move the table to
match the picture) is stated and justified by the Part C lookup - I agree with it.

### Findings

**RV-1 (major) B3 F21: "hip height" is a correct point, but not a defined robot state, and there is no plausibility check.**
- Trigger: B3 says "measured with a tape while the robot stands" in Part B, in whatever mode the owner placed the robot. Part C runs the
  table steps with the robot standing "in the arm_sdk mode agreed with the owner" (Page 1 gantry definition; M0 says the owner switches
  to that mode). The G1 locomotion controller's stand height depends on mode and can be changed from the remote, so the hip height in
  B3 can differ from the one at M4-M6 by centimetres. The formula then carries that error 1:1 into the table height - the very error
  F21 exists to prevent. Also nothing catches a gross mis-measurement (pelvis top / pelvis origin instead of the hip axis: +10 cm, the
  exact old bug).
- Evidence: B3 lines 251-253 (working tree); Page 1 "hangs in the gantry"; M0 text; no re-measure anywhere in Part C.
- Fix (text): "Measure with the robot standing in the mode it will have at the table in Part C (ask the owner to set it now and not
  change the stand height later); if the owner changes mode or height, measure again and re-set the table." Add "Hip height is
  expected near 0.65 m; if your number is below 0.60 or above 0.72 m, measure again; still outside: ask the owner and write it down -
  do not set a table higher than 0.84 m." Name the landmark: "the centre of the round motor cap on the side of the hip, the upper-most
  leg motor, seen from the side" (hip roll is 3 cm lower; confusing them costs 3 cm).

**RV-2 (major) B3/F3: only the table is marked, not the robot - yet Part C's lookup assumes the 24.5 cm still holds.**
- Trigger: B3 tapes the table's front corners; Part C M4 says "Put the table at the tape mark from the lookup". Between B and C the owner
  may walk/re-place the robot (gantry, mode change, breaks Q9). Then the table is on its marks but no longer 24.5 cm from the hip axis,
  and 87's clearance (computed for that offset) silently does not apply.
- Fix (text): B3: "also mark both feet (toe and heel) with tape"; and a sentence for Part C (owner of Part C to fold in): "before M4,
  check the robot's feet are on their marks and the 24.5 cm (+ setback) once more."

**RV-3 (major, coordinator add-on) B3: the head is motorized, but the runbook tells the operator to "set the D455" as if by hand, and
never mentions pan.**
- Trigger: B3 "Set the D455 to 50.0 +- 1 deg below horizontal: lay a phone flat on the D455's top face" and later "correct them (the
  camera only within 49-51 deg)". An operator will push the camera by hand: against a powered servo that can strip/overload it, or the
  module returns to its own setpoint; and the pan axis (XL330, +-50 deg, `g1sim/payload.py:6`) is never mentioned although training has
  pan 0 (`table_scene.HEAD_POSE`). Q3 asks only "is the mount tilted".
- Fix (text): "The head module is motorized: never move it by hand. The owner sets it with the head's own control (app/remote) to
  straight ahead (pan 0) and 50.0 deg down (tilt ~48.7 deg on the module + 2.3 deg mount - 1 deg pelvis, PLAN.md 2.4). You only check:
  phone flat on the D455 top face reads 49-51 deg, and the 84 overlay shows the table centred left-right." Change Q3 (paper list +
  `real_bringup.QUESTIONS`, needs code) to: "Head module: how is it set (app/remote), can it be set to pan 0 / 50 deg down and left
  there, and what else can move it (behaviours, apps, BotBrain)?"

**RV-4 (major, coordinator add-on) A+B and how Part C relies on it: nothing stops or detects a head move between the 84 camera check and
the end of M4-M6.**
- Trigger: owner app, a robot behaviour, BotBrain or a mode change (M0 switches mode; a head may re-home) moves the head after B3. M4
  records "real data" and M5/M6 feed the frames to the policy; a moved head silently changes the policy's input (different view than
  training) and the recorded data.
- Evidence: no runbook rule mentions the head; no `84 --steps camera` re-run before M4/M5/M6; 86's camera preflight checks only frame
  age (`g1sim/real_preflight.py:158-161`, "no camera frame younger than 0.5 s"), no framing/overlay check; 84's check is by eye only
  (real_bringup.py:196-217). No code commands the head: grep of `g1sim/real_*.py`, `motion_gate.py`, `scripts/84_*`, `scripts/86_*` for
  head/xl330/dynamixel/set_head finds nothing (head control exists only in sim: payload.py, table_env.py, 51/53 build scripts). So our
  code never moves it - but also never notices if something else does. Whether the head angle is visible on DDS/lowstate: not checked
  (unknown, no robot).
- Severity reasoning: not an arm-safety issue (87's table clearance does not depend on the camera; 86's envelope/safety filter still
  bound the arms), but it voids M4-M6 results and makes M6 a policy acting on an unseen view. Major.
- Fix (text, B3 + Part C): "After B3 passes, the owner must not move the head and turns off anything that moves it (head tracking,
  behaviours, apps, BotBrain). Re-run the B3 84 camera line and compare the overlay with the B3 one before M4, before M5, and after any
  mode change or reboot; different: the owner re-sets the head (RV-3), then check again." Optional code (push 2 or later): 86 compares
  its first frame with the B3 overlay frame (e.g. image shift/correlation) and refuses shadow/run beyond a threshold.

**RV-5 (minor) B3 reading of `72 kamera --suchen`.**
- "the last line: it must start with `opencv`" - the last line printed is actually the German hint (`print(ssh(...))` then
  `print("Das Farbbild ist ...")`, 72:204-205). Say "the last line before the German hint". "the first line: one with `8086`": there can
  be several 8086 lines; the D455 is `8086:0b5c`. Also if the ssh login fails, 72 prints ssh's error and an empty list followed by the
  hint - say "no `/dev/video` lines at all: login problem, redo the ssh line".

**RV-6 (minor) B3 `it must run at 1280 x 800` -> "not the colour stream - try the next number".** On the right YUYV node this also
appears
when the D455 is on USB 2 (Q2). Add: "if the `YUYV` node gives it, ask the owner whether the D455 is on USB 3."

**RV-7 (minor) B3 manual ssh line has no connect timeout.** `ssh unitree@192.168.123.164 echo ok` without `-o ConnectTimeout=5` can hang
a minute or more before `Connection timed out` (72/84 use 5 s). Use `ssh -o ConnectTimeout=5 unitree@192.168.123.164 echo ok`.

**RV-8 (minor) B3 table placement wording.** (a) "volleyball on its stand 14.4 cm back" - say "the stand's centre" (the basket line says
"centre"). (b) "24.5 cm in front of the hip joint axis (measured horizontally)" - the axis is in the air at ~0.65 m: suggest "hang a
string with a weight from the hip motor centre, tape the floor point, measure 24.5 cm along the floor to the point below the table's
front edge". (c) "measure the 24.5 cm ... once more and correct them" vs "do not move the table off its tape marks": say "if the 24.5 cm
was wrong, move the table and re-tape the marks; after that, the marks are fixed".

### Checks I could not run
- Anything on the real robot: head module control path, whether a mode change re-homes the head, D455 v4l2 format names on the robot PC,
  the real hip height range, SSH login.
- The Ubuntu install path end to end (only the apt block text and gh versions were verified).

### Verdict
- Implementation fidelity: **GO** - the fixes do what the list says; flags and quoted strings match; test gate 18 passed, 1 skipped.
- Plan correctness of the fixes: F21 formula and F3 numbers are **right** (re-derived); F1 command is read-only; F2/F4/F5 correct.
  But the measurement state (RV-1), robot position (RV-2) and the motorized head (RV-3, RV-4) are open.

**NO-GO for push 1 as the text stands** - fold RV-1, RV-2, RV-3 and the text half of RV-4 into B3 (about 10 lines, text only; RV-2's and
RV-4's Part C sentence go to whoever owns Part C); the minors can ride along. Then GO without another full review (a diff check of those
lines suffices). RV-4's optional 86 framing check and the Q3 code change stay OPEN for push 2.

## Fixes for the review of dry-walk A+B fixes

Fixer: implementer agent (opus), 2026-10-18. Changed: `docs/ROBOT_DAY_RUNBOOK.md` (Part A sheet fields 52-54, Part B intro 199-202,
B1 235-248, B3 258-342, Part C M0 398-400, M4 478-495, M5 505-510; line numbers of the working tree after this edit),
`tests/helpers/loopback_lowstate_pub.py` (one line), `tests/test_unit_runbook.py` (FakeClock jitter + hand-rate assert).
Gate: `~/dev/g1-ext/.venv/bin/python -m pytest tests/test_unit_runbook.py -q` -> 18 passed, 1 skipped (87 report not written yet),
full output `runs/real_runbook_rv_fix.log`. Facts from Finnegan used: the head camera is motorized and the robot can move it; the
robot PC login exists only on site at SAP, from the owner.

- **RV-1 (major) hip height state, landmark, sanity range:** B3 now starts with "Robot state first": the owner puts the robot in
  the mode it will stand in at the table in Part C (Q8) and leaves mode and stand height alone; if the owner keeps another mode for
  now, measure again after M0's mode switch. Landmark re-derived from the MJCF (`scene_29dof_brainco_volleyball_walk.xml:159-166`):
  hip pitch is the top-most leg joint, axis left-right (`axis="0 1 0"`); hip roll sits 3.0 cm lower and 5.2 cm further out
  (`pos="0 0.052 -0.030465"`), axis forward (`axis="1 0 0"`) - "do not measure to it"; unsure -> the owner points at the hip pitch
  motor. Measured with a plumb string from that motor (also gives the floor "hip point", RV-8b). Range: near 0.65 m; outside
  0.60-0.72 m -> measure again (pelvis centre is 10.3 cm higher = the old F21 mistake); still outside -> the owner points at the
  motor, measure there, write "owner checked". Derived table range 0.69-0.81 m stated. **Deviation:** the reviewer's "never set a
  table higher than 0.84 m" was not adopted - 0.84 is not derivable from the formula or the range (0.72 m hip gives 0.81 m), and
  with the owner-checked measurement the formula is right by construction. Part C: M0's repeat line and the new "Robot and head
  check" (M4 step 1) re-measure the hip height and re-set the table if it moved more than 1 cm.
- **RV-2 (major) robot not marked:** B3 tapes both feet (toe and heel). Part C, new "Robot and head check" step 1 (before M4,
  before M5, after any mode change/reboot): feet on their marks and the hip point 24.5 cm (within 1 cm) behind the front edge at
  the Part B mark; off -> the owner re-places the robot with the remote (never push it), still off -> skip M4-M6 at the table. M4's
  table placement now says the distance becomes 24.5 cm + N cm at a "back N cm" mark. The 1 cm tolerance is my choice (the runbook
  had no number; same order as the plumb/tape reading) - open for Finnegan to change.
- **RV-3 (major, motorized head):** B3 "Head camera" bullet: never move it by hand; the owner sets it with the head's own control to
  straight ahead and 50.0 deg down (48.7 tilt + 2.3 mount - 1.0 pelvis, PLAN.md 2.4: 48.70 + 2.29 - 0.96); operators only check
  phone 49-51 deg (without pressing) and the overlay shows the table centred left-right; off -> the owner corrects it. The overlay
  paragraph no longer says "correct the 50 deg" by hand: the owner corrects with the head's control. B1: when 84 asks Q3, also ask
  how the head is set, whether it can stay at straight ahead / 50 deg down all day, and what else can move it; the paper Q3 points
  to it. The Q3 text change in `real_bringup.QUESTIONS` (code) stays OPEN for push 2, as the reviewer said.
- **RV-4 (major, no head move after B3):** B3 "The head stays put after B3": the owner turns off everything that can move the head
  (head tracking, behaviours, apps, BotBrain) and leaves its control alone; if it moves anyway, re-set + re-run the 84 camera line
  + compare. The overlay file name now always goes on the sheet (Part A sheet list too). Part C: M4 "Robot and head check" step 2
  re-runs `84 --steps camera` (the B3 line, as a code block) with the table on the Part B marks and compares the new overlay with
  the B3 one; different -> the owner re-sets the head, run again; still different -> skip M4-M6, write it down. M0: after its mode
  switch and every later mode change or reboot, do the check before the next M4-M6 line. M5: do the check once more (also if M4
  was skipped). Design choice: the comparison is done with the table at the Part B mark, because at a setback mark the table edge
  legitimately moves in the picture. The optional 86 framing check (code) stays OPEN for push 2.
- **RV-5 (minor) 72 output:** D455 = the `8086:0b5c` line (others are other Intel devices); `opencv` is "the last line before the
  German hint" (72:229-230 print the ssh output, then the hint); "no `/dev/video` lines at all" = login problem (72's `ssh()`
  prints ssh's stderr and returns empty stdout, 72:174-176) -> redo the ssh line.
- **RV-6 (minor) 1280 x 800:** if the `YUYV` device gives it, ask the owner whether the D455 is on USB 3 (Q2), move it, run again.
- **RV-7 (minor) ssh timeout:** manual line is now `ssh -o ConnectTimeout=5 unitree@192.168.123.164 echo ok` (72 and real_camera
  use 5 s too).
- **RV-8 (minor) wording:** (a) "the stand's centre (with the volleyball on it)"; (b) plumb string from the hip pitch motor, tape
  the hip point, measure along the floor to the point under the front edge; (c) "If the 24.5 cm turns out wrong later in B3, move
  the table and re-tape the marks; after B3 the marks are fixed", and the overlay paragraph says the same.
- **84 segfault + hand rate (coordinator add-on):** Part B intro bullet with the given text (plus "copy the whole terminal text of
  both runs into a note" so "keep the log" is doable). Facts re-derived: STEP lines print before `write_report` (84 main), the
  segfault trace comes from `faulthandler.enable()` (84:278); `HAND_RATE_LISTEN_S = 1.0` (84:40, 144); `HAND_STATE_MIN_HZ = 50.0`
  warning only (real_bringup.py:20, 194).
- **Loopback hand drift:** `run_loop` now `next_hands += 1.0 / HAND_HZ` (fixed schedule, like `next_tick`). The old test could not
  see the drift (fixed 1.2 ms overshoot: hands 96.6 Hz both before and after). `test_loopback_pub_keeps_rate_when_publish_is_slow`
  now jitters the overshoot (0.6/1.2/1.8 ms, same 1.2 ms mean; FakeClock accepts a tuple) and also asserts hands >= 95 Hz.
  Mutation check: with the old line the test fails `assert 66.7 >= 95.0` (`runs/real_runbook_rv_mutation.log`); with the fix
  lowstate 200 Hz, hands 100.0 Hz.

Still OPEN (not text): Q3 wording in `real_bringup.QUESTIONS`, 86 first-frame framing check (both push 2); real-robot facts the
reviewer could not check (head control path, re-homing on mode change, real hip height).

## Diff check of RV fixes

Reviewer: independent reviewer agent (opus), 2026-10-18 18:48-18:55. I did not write the runbook, the RV list or the fixes and have
not seen the fixer's reasoning beyond "Fixes for the review of dry-walk A+B fixes" above. Scope: `docs/ROBOT_DAY_RUNBOOK.md`
lines 48-56, 191-204, 231-248, 258-342, 398-400, 475-514 (working tree), `tests/helpers/loopback_lowstate_pub.py` `run_loop`,
`tests/test_unit_runbook.py` FakeClock + `test_loopback_pub_keeps_rate_when_publish_is_slow`. No files edited.

### What I re-derived / ran
- **Gate:** `~/dev/g1-ext/.venv/bin/python -m pytest tests/test_unit_runbook.py -q` -> **18 passed, 1 skipped** (87's
  `arm_path_clearance.json` not written yet), exit 0, `runs/real_runbook_rv_review.log`. The loopback tests ran (the g1-ext venv has
  `unitree_sdk2py`; the only skip is the clearance test).
- **Mutation (own script, no repo edit):** loaded the helper source, swapped `next_hands += 1.0 / HAND_HZ` back to
  `next_hands = now + 1.0 / HAND_HZ`, ran the test function against it: **fixed PASS; old line FAIL "only 67 Hz hand state
  published, asked for 100 Hz"**. Old constant-overshoot clock against the old line: hands 96.6 Hz (the old test could not see it,
  as the fixer said); fixed: 100.0 Hz. `runs/real_runbook_rv_review_mutation.log`. The publisher change is correct and matches
  `next_tick` scheduling. After a 0.5 s stall the fixed schedule catches up by sending hands on every (also catching-up) lowstate
  tick (24 per 50 ms window) - same behaviour as `next_tick`, loopback only, harmless (info).
- **Hip landmark (MJCF `scene_29dof_brainco_volleyball_walk.xml:159-166`):** hip pitch body is the first leg body under the pelvis,
  `axis="0 1 0"` (left-right); hip roll `pos="0 0.052 -0.030465"`, `axis="1 0 0"`: "3 cm lower, 5 cm further out, axis forward" is
  correct. Pelvis origin 0.1027 m above the hip axis -> "pelvis centre 10 cm higher" correct.
- **Ranges:** 0.74 + (0.60 - 0.648) = 0.692, 0.74 + (0.72 - 0.648) = 0.812 -> "0.69-0.81 m" correct; "table top 9 cm above the
  hip axis" = 0.0922 correct.
- **Head:** PLAN.md 2.4 (line 247-248): "50.0 deg below horizontal (tilt 48.70 + mount 2.29 - pelvis 0.96)"; runbook "48.7 + 2.3
  - 1.0" = 50.0, correct. PLAN.md 4.1 (3) (lines 648-656): owner sets with head's own control, operators never move it, phone
  49-51 + overlay, re-run 84 camera before M4, before M5, after any mode change/reboot - the runbook says the same (B3 287-291,
  M0 398-400, M4 478-491, M5 505-506).
- **D455 USB id:** `IntelRealSense/librealsense` `src/ds/d400/d400-private.h:40` (fetched today via gh api):
  `const uint16_t RS455_PID = 0x0B5C; // D455` -> `8086:0b5c` correct.
- **72 / ssh facts:** `72_real_check.py:153-155` (CAMERA_FIND: lsusb grep 8086, one line per /dev/video*, opencv last),
  `:171-176` (`ssh -o ConnectTimeout=5`, prints stderr only when rc != 0 and no stdout), `:229-230` (ssh output, then German hint);
  `real_camera.py:34` `SSH_CONNECT_TIMEOUT_S = 5`, `:29` `INFO_TIMEOUT_S = 30.0`; runbook line 297
  `ssh -o ConnectTimeout=5 unitree@192.168.123.164 echo ok` correct.
- **84 facts:** `HAND_RATE_LISTEN_S = 1.0` (84:40,144), `HAND_STATE_MIN_HZ = 50.0` warning only (real_bringup.py:20,186-195),
  `faulthandler.enable()` (84:278), `write_report` after the STEP lines (84:270) - the Part B intro bullet is correct.
- **Part C consistency:** the new M4 code block (`84 --steps camera ...`) comes after M0, so
  `test_runbook_part_c_starts_with_factory_pose_record` still holds; M0 (repeat on mode change + check before next M4/M5/M6), the
  "Robot and head check" (before M4, before M5, after mode change/reboot) and M5 ("also if M4 was skipped") agree with each other and
  with B3. Step 1 (re-set table height) precedes step 2 (camera), so a legitimate stand-height change keeps camera-to-table geometry
  and the overlay still matches - consistent. "back N cm -> 24.5 cm + N cm" matches the lookup's "back N cm to the second mark"
  (test `expected_cells`); 87 is relative to the hip axis, so nothing contradicts the lookup. 1 cm tolerance is inside 87's 5 cm
  `CLEARANCE_MIN_M` margin (PLAN.md 3.5).
- **No password:** grep of the runbook, publisher and test for passw/Passwort/123: only the word "password" (admin password,
  "never the password", the ssh prompt text); no value anywhere.

### Implementation fidelity
**GO.** RV-1 (state first, landmark, 0.60-0.72 range, re-measure on mode change), RV-2 (feet toe+heel marks, Part C 24.5 cm within
1 cm check), RV-3 (motorized head, owner-only, 50.0 deg derivation, Q3 add-on question), RV-4 (no head move after B3, re-check before
M4/M5/after mode change or reboot, overlay file name on the sheet), RV-5 (8086:0b5c, last line before the German hint, no
/dev/video lines = login), RV-6 (USB 3), RV-7 (ConnectTimeout=5), RV-8 a/b/c - all present and correct against code, MJCF and PLAN.md.
One stated deviation (the 0.84 m cap not adopted) - see D1.

### Findings

**D1 (minor) B3 hip height: the "owner checked" path accepts any number, and the dropped 0.84 m cap deviates from PLAN.md.**
- Trigger: hip reading above 0.72 m twice (e.g. pelvis again), owner points at a motor, operator writes "owner checked" and sets
  table = 0.74 + (hip - 0.648), e.g. 0.85 m for a 0.76 reading. Line 269-270.
- Evidence: PLAN.md task RB item 2 (line ~911) says "never a table above 0.84 m"; the fixer dropped it as "not derivable". But the G1
  hip pitch axis cannot be above ~0.68 m with feet on the floor: sum of the leg chain's z offsets with straight legs (MJCF 164-187:
  0.0305 + 0.1241 + 0.1773 + 0.3000 + 0.0176 + 0.035 sole) = 0.685 m. So a reading above 0.72 m is always a mis-measurement, and a
  too-high table is the collision direction (closer to the arms). The fixer's "with the owner-checked measurement the formula is
  right by construction" only holds if that measurement is right - exactly what the cap guards.
- Reach: not reachable on push 1 (Part C keeps "M4-M6 at the table are NOT cleared"); reachable once 87's rows are copied in.
- Fix (one line): "Still above 0.72 m after the owner pointed: write it down and do not set the table above 0.81 m (skip M4-M6 at
  the table)"; or keep the text and record the dropped cap in PLAN.md RB item 2. Low-probability (two failed measurements + owner),
  slow replay in the gantry with contact as an abort - minor.

**D2 (minor) B3 72 output: a failed login hits the "no OpenCV - skip B3 and B4" bullet first.**
- With a failed login, `ssh()` prints ssh's stderr, `print(ssh(...))` prints an empty line, then the hint (72:174-176, 229-230). The
  "last line before the German hint" is then empty -> line 318-319 "Anything else: ... no OpenCV - tell the owner; skip B3 and B4",
  which comes before the login bullet (line 320). Fix: move the "no `/dev/video` lines at all" bullet first, or add "(only if
  `/dev/video` lines are shown)" to the opencv bullet.

**D3 (minor) Part C check step 1: re-placing the robot to within 1 cm with the remote is unlikely; the miss fails safe but costs the
table steps.**
- Line 480-482: off by > 1 cm -> the owner walks the robot back onto its marks; still off -> skip M4-M6 at the table. G1 stepping by
  remote is not cm-precise. 87 is relative to the hip axis, so the equivalent fix is to move the table (with stand and basket) back
  to 24.5 cm / centred and re-tape the marks, and use the owner only if the robot is turned. Fails safe (skip), so minor; Finnegan's
  call (the fixer also left the 1 cm open for him).

**D4 (minor) B3 head check wording: "a phone laid flat on the D455's top face (do not press)"** (line 285). The top face is tilted
50 deg and only ~26-29 mm deep: a phone does not stay on it unheld. Fix: "hold a phone lightly flat against the D455's top face
(do not push the head)". (The phone method itself is PLAN.md 2.4.)

**D5 (info, process) Part C was edited by the RB fixer, but PLAN.md task RB lists "Part A, Part B, end of day only" and gives Part C to
E2's fix round (next writer).** The edits are what RV-2/RV-4 asked for and are consistent (above), but E2's fix round must start from
this working tree (M0 398-400, M4 478-495, M5 505-506) and not overwrite them; update RB's `<files>` line or tell E2.

**D6 (info) Head set value:** "set ... to 50.0 deg down (in the simulator: 48.7 deg head tilt + ...)" may make the owner dial 50 on
the module's own scale (about 51.3 world). The phone 49-51 check catches it; optional: "the phone reading counts, not the head's
scale".

### Checks I could not run
- Real-robot facts: head control path and whether a mode change re-homes it, real hip height and the hip pitch motor's look, D455
  enumeration and v4l2 format names on the robot PC, ssh behaviour against the real robot PC, remote re-placing precision (D3).
- 72 and 84 against a robot PC (no ssh target); D2 is derived from the code, not from a live run.

### Verdict
- Implementation fidelity: **GO** (all RV-1..RV-8 fixed as listed; publisher fix correct; test catches the old line: 67 Hz FAIL).
- Plan correctness of the fixes: **right** (landmark, range, formula, head angle, D455 id, ConnectTimeout, Part C order re-derived);
  D1-D4 are minors, D5-D6 info.
- **Overall: GO for push 1.** D2 and D4 are one-line text fixes that can ride along; D1 should be fixed (or PLAN.md updated) before
  87's rows replace the NOT cleared line in push 2; D3 is Finnegan's call; D5 must reach E2's fix round.

## Main-session fixes after the diff check (2026-10-18 18:53)
- D1: "still above 0.72 m" now caps the table at 0.81 m and skips M4-M6 at the table.
- D2: the login-problem bullet moved before the OpenCV bullet.
- D4: phone "held lightly flat against" the top face.
D3 (re-place robot within 1 cm vs. move table) left for Finnegan; D5/D6 info only. Unreviewed text edits - final gate re-checks them.
