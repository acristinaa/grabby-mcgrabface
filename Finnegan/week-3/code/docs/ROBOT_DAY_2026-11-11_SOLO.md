# Solo robot day, Wednesday 2026-11-18 (SAP Potsdam)

(Moved from 2026-11-11; the file name keeps the old date so the links in PLAN/PROGRESS/reviews stay valid.)

For Finnegan, alone with the SAP robot owner. One page to work from. The details ("if you see ...") stay in
`docs/ROBOT_DAY_RUNBOOK.md`; this page says the order, the solo parts and the exact lines.

Every terminal starts with these two lines (the canonical repo, not `~/UnitreeG1-Robot`):
```
cd "$HOME/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot"
source ~/g1-robot-day/robot-env.sh
```
The MacBook Pro M4 is both the robot laptop and the policy laptop. Model: **run3 150k** (trained on the 1,691 v2 demos; sim eval 68/100 over v1+v2 ranges vs run1 72 % on v1 only),
copied from the Hive PC to `~/g1-robot-day/smolvla-runs/run3/checkpoints/150000/pretrained_model` on 2026-11-16 (sha256
`ef86fffe...db931`, same as on the PC). The demo envelope in `g1sim/real_safety.py` covers v1 and v2.

## What the robot will do tomorrow, and what not

- **Will:** read-only bring-up (Part B, nothing moves). Then, behind the motion gate: arms up to the ready pose and back
  (M2), a slow replay of one recorded demo in the air (M3). Only if the S1 check says clear and M3 at waist 0.6 was clean: the
  same replay at the table (M4), the VLA watching without moving (M5), and the VLA trying to pick up the ball (M6: speed 0.25 for
  at most 180 s, then 0.5 for at most 90 s, both 45 s of policy time; the done check ends an attempt right after a place,
  runbook M6). A miss is fine.
- **Will not:** walk (the owner's remote only), use any waist setting other than 0.6 at the table, use `86 recover`, use the
  arms-up-first layout, train anything, use run1.
- **Still unproven (say it to the owner):** how far the real torso leans forward under the arms (O1; in the 85 sim it leaned
  more than in training, and then the fence held the replay); the fence knows the table layout only to about 1-2 cm (tape
  measure), so it is a seatbelt, not a "never touches" promise; the policy has only ever seen the simulator (run3: 68 of 100
  sim starts, 60 % on the harder v2 scenes with the fence on); what the G1 does when our messages stop (Unitree does not publish it).

## 1. Tonight, in this order

1. **Wait for the setup.** The setup now running into `~/g1-robot-day` must end with these three lines:
   `SETUP robot-env: OK ...`, `SETUP release: OK model in ...`, `SETUP policy-env: OK ...`. A `FAILED` line: do what it says
   and run the same setup command again (finished blocks are quick).
2. **Firewall (runbook A4).**
```
/usr/libexec/ApplicationFirewall/socketfilterfw --getglobalstate
```
   "disabled": nothing to do. "enabled": run the first block of runbook A4 (the robot env's Python, four `sudo` lines). The
   policy-env block is not needed: the policy server runs on this Mac and listens only on 127.0.0.1.
3. **Self-test 89** (robot cable and Ethernet adapter unplugged; loads the model once, 1-5 min, the Mac gets warm):
```
source ~/g1-robot-day/robot-env.sh
caffeinate -i python scripts/89_robot_day_selftest.py
```
   PASS looks like: one `CHECK <name>: PASS ...` line for versions, dds, camera, bringup, imports, fence and policy, then
   `CHECK motion: INFO MOTION: LOCKED` (normal tonight), then `SELFTEST: PASS`. In the `CHECK policy` line find `chunk_ms N`:
   **N must be below 1500** (your first run: 1318). 1500 or more: run 89 once more. Still 1500 or more: no M5/M6 tomorrow.
4. **The USB-C to Ethernet adapter.** Plug it in (no cable needed), then:
```
bash scripts/setup_robot_laptop.sh --show-ip
networksetup -listallhardwareports
```
   The second line must list the adapter (for example `Hardware Port: USB 10/100/1000 LAN`, `Device: en5`); that `enN` is
   your `<IF>` tomorrow - write it on the sheet. Now give it the fixed address as in runbook B0 (System Settings > Network >
   the adapter > Details > TCP/IP > Manually, 192.168.123.222, mask 255.255.255.0). macOS may print its address in the
   `--show-ip` list only once a cable has a link. Unplug the adapter again before any 89 run (89 fails while a 192.168.123.x
   address exists).
5. **The motion unlock** - only after the code commit of this solo build exists, and only if you accept section 5. The gate
   reads the unlock from the committed file, pins one full commit id, and accepts it only on the date written in it:
```
git status --porcelain -- g1sim scripts tests
git rev-parse HEAD
```
   The first line must print nothing (every file 86 loads is committed). The second prints the full 40-character id: that is
   `<FULL COMMIT ID>`. The file content, exactly as the gate reads it (`g1sim/motion_gate.py`):
```
{"motion_allowed": true, "commit": "<FULL COMMIT ID>", "unlocked_by": "finneganadam", "date": "2026-11-18"}
```
   Write it, commit it and check it with these lines (they fill in the id themselves):
```
MOTION=$(git rev-parse HEAD)
printf '{"motion_allowed": true, "commit": "%s", "unlocked_by": "finneganadam", "date": "2026-11-18"}\n' "$MOTION" > docs/ROBOT_DAY_MOTION.json
cat docs/ROBOT_DAY_MOTION.json
git add docs/ROBOT_DAY_MOTION.json
git commit -m "Unlock motion for the solo robot day 2026-11-18 (accepts the default rulings in docs/ROBOT_DAY_2026-11-11_SOLO.md section 5)"
python scripts/86_real_run.py gate
```
   Tonight the last line prints `MOTION: LOCKED` and exactly one problem line:
   `  the unlock is for 2026-11-18; only Finnegan can unlock today`. That one line is the proof that the file is committed,
   readable, by you, and pins a commit of this checkout; the date is the only thing missing. (The date check stops the gate
   before it compares the loaded files; the empty `git status` line above covers them tonight.) Any other line: fix what it
   says. Tomorrow morning the same command prints `MOTION: UNLOCKED` (tried in a throw-away clone on 2026-11-10).
   **Any later edit or commit of a file 86 loads (g1sim, scripts, the fence model) locks motion again** until you pin again.
   Editing PROGRESS.md or other docs does not. Push is optional: the gate reads this local repo.

## 2. Packing list

- MacBook + charger + power strip
- USB-C to Ethernet adapter, 2-3 m Ethernet cable
- phone with a level app + small tripod (video of every motion step)
- tape measure, string + small weight, painter's tape, marker
- paper sheet + pen ("the sheet": `<IF>`, FACTORY ARMS line, Q8/Q10 answers, hip and table height, overlay file name, camera
  number N, robot PC user@IP (never the password), S1 result, each 86 result line)
- volleyball, a light ball stand, the basket if there is one

**Ask SAP / the owner before you go:**
- a gantry with a harness for the G1 (the robot stands on its feet, harness with only a few cm of slack)
- the owner at the remote (with the e-stop) the whole time
- a table 90 x 65 cm whose top can be set between 0.69 and 0.81 m
- Q8: is `rt/arm_sdk` allowed on this firmware, and in which robot mode (remote combo)?
- Q10: which firmware version runs on the robot?
- the robot PC login (user + password, only on site; usually `unitree@192.168.123.164`)
- power sockets near the robot

## 3. On site, step by step

Roles, all day: **you** at the MacBook, one hand on Ctrl+C; **the owner** at the remote with the e-stop; **the phone** on
the tripod, filming every motion step. You are alone at the laptop, so nobody else reaches in: say out loud before M1 how an
86 run ends (runbook Part C, "How an 86 run ends").

**Flight recorder (added 2026-11-18, read-only):** from B0 on, keep a second terminal open with
`source ~/g1-robot-day/robot-env.sh && python scripts/91_watch.py --iface <IF>`. It records once per second which layer is
healthy - NET (cable/ping), STATE (rt/lowstate), HANDS, WRITERS (a second program on rt/arm_sdk = possible fight, ARM confirms it; api request /
cmd_vel senders = the navigator), ARM (arm not following our command), BASE (robot walking or tilting) - prints an `ALERT`
line the moment one goes wrong and a summary at Ctrl+C. When something fails, the earliest ALERT time names the layer. The
file lands in `runs/watch/`; send it to Finnegan. It never sends anything and does not open the camera (84 checks that).

**What 86 asks before every motion command (solo):**
```
Operator 1, type your GitHub name (...)          -> you type:        finneganadam
Operator 2, type your GitHub name - or, ...      -> the owner types: OWNER   (only while holding the remote)
Both operators agree to start motion? ...        -> START
Robot stands on its feet, harness on the gantry ... -> HANGING   (only in that state, runbook page 1)
```

| About | Step | What you do (details: runbook) |
|---|---|---|
| 0:00 | arrive | owner questions, gantry, table. `python scripts/86_real_run.py gate` -> `MOTION: UNLOCKED` |
| 0:10 | B0 | robot cable in, `--show-ip`, ping, `84 --steps net,probe,joints`; FACTORY ARMS line on the sheet |
| 0:20 | B1 | `84 --steps questions`; Q8 and Q10 must be answered |
| 0:30 | B2 | `84 --steps hand-state` |
| 0:35 | B3 | robot mode for the day, feet tape, hip height, table height and position, head 50 deg, ssh login, camera N, overlay |
| 1:05 | B4 | `84 --steps record` for 3-5 layouts, then back to the B3 placement |
| 1:20 | M0 | owner switches to the arm_sdk mode; `84 --steps joints --record-factory-pose --mode-note "..."` |
| 1:25 | S1 check | bounds line, then 87 on `runs/factory_pose_robot.json` (exact line below); `S1: clear` or `not clear` on the sheet |
| 1:30 | M1 | `86 release` - nothing may move |
| 1:35 | M2 | table moved away; `86 ready --hold 5` |
| 1:40 | M3 | second terminal: replay server (`81 --replay ... --port 8766`); lines 1-3; line 4 (waist 0.6) only with the owner's yes |
| 1:55 | check | robot and head check (runbook M4): feet on the tape, hip point 24.5 cm, camera overlay as in B3 |
| 2:05 | M4 | only if `S1: clear` and M3 line 4 clean: table on its tape marks, stand 6.7 cm further away and 1.7 cm right, replay at waist 0.6 (line below) |
| 2:10 | go/no-go | `FENCE SUMMARY`: holds 0, table >= 1.31, stand >= 2.54, basket >= 2.02 cm, else no M6 at the table; stop the replay server |
| 2:15 | M5 | robot and head check again; third terminal: policy server (below), wait for `LISTENING 8765`; `86 shadow` |
| 2:25 | M6 | at least 3 filmed attempts, first speed 0.25, then 0.5 (lines below) |
| end | runs | section 6 |

The S1 line (after every M0; about 45 s; nothing moves):
```
~/dev/g1-ext/.venv/bin/python scripts/87_arm_path_clearance.py --waypoint-paths --factory-pose runs/factory_pose_robot.json --report runs/safe_paths_onsite.json --demo tests/data/replay_demo.npz --waist-limits 0.6 --setbacks 0.0
```
Clear = the `elbow ...` line says `way in clear`, `recover from the way in clear` and `way out per waist: 0.60 clear`, and
`lift home waist 0.60: 5265/5265 pass`. Anything else, a `ValueError ... skip table steps`, or no `REPORT` line: no M4 and no M6
at the table; M5 and M6 only with the table moved away. (What both cases look like: runbook, "S1 check".)

M3 replay server, in a second terminal (`cd` line and `source ~/g1-robot-day/robot-env.sh`, then; leave it running until
after M4; it only plays back the recorded demo, no model):
```
python scripts/81_smolvla_server.py --replay tests/data/replay_demo.npz --port 8766
```
Then the M3 lines 1-4 from runbook M3 in the first terminal (they need no path change).

Table steps only if the table top is within -4 .. +1 cm of the B3 target height (`--fence-table-dz-cm` -4 .. 1) and the
robot stands straight on its marks: S1 checked only those cases. Otherwise re-set the table, or skip the table steps.

M4 at the table (replay server running on 8766; `N` = camera number; change the table-dz and basket numbers only if the sheet
says so):
```
caffeinate -i python scripts/86_real_run.py replay --iface <IF> --server 127.0.0.1:8766 --episode replay_demo --robot-pc unitree@192.168.123.164 --camera-device N --speed 0.5 --waist-max 0.6 --fence-setback-cm 0 --fence-table-dz-cm 0 --fence-stand-dx-cm 6.7 --fence-stand-dy-cm -1.7 --fence-basket-dx-cm 0 --fence-basket-dy-cm 0
```
Policy server for M5/M6, in a third terminal (`cd` line, then):
```
source ~/g1-robot-day/policy-env.sh
caffeinate -i python scripts/81_smolvla_server.py --checkpoint ~/g1-robot-day/smolvla-runs/run3/checkpoints/150000/pretrained_model --device mps
```
M5 and M6 at the table, with the stand back on its B3 mark (first terminal):
```
python scripts/86_real_run.py shadow --iface <IF> --server 127.0.0.1:8765 --robot-pc unitree@192.168.123.164 --camera-device N --max-seconds 30 --waist-max 0.6 --fence-setback-cm 0 --fence-table-dz-cm 0 --fence-stand-dx-cm 0 --fence-stand-dy-cm 0 --fence-basket-dx-cm 0 --fence-basket-dy-cm 0
caffeinate -i python scripts/86_real_run.py run --iface <IF> --server 127.0.0.1:8765 --robot-pc unitree@192.168.123.164 --camera-device N --speed 0.25 --max-seconds 180 --waist-max 0.6 --fence-setback-cm 0 --fence-table-dz-cm 0 --fence-stand-dx-cm 0 --fence-stand-dy-cm 0 --fence-basket-dx-cm 0 --fence-basket-dy-cm 0
caffeinate -i python scripts/86_real_run.py run --iface <IF> --server 127.0.0.1:8765 --robot-pc unitree@192.168.123.164 --camera-device N --speed 0.5 --max-seconds 90 --waist-max 0.6 --fence-setback-cm 0 --fence-table-dz-cm 0 --fence-stand-dx-cm 0 --fence-stand-dy-cm 0 --fence-basket-dx-cm 0 --fence-basket-dy-cm 0
```
With the table moved away (S1 not clear, or no clean M3 line 4): the same three lines with the last clean M3 waist setting
instead of `--waist-max 0.6`, and only `--fence-setback-cm 100` instead of all the fence numbers.

## 4. Stop rules (one list)

- The owner says stop: stop. The owner and the remote always win.
- Any jerk or swing, a pose that looks mirrored, the robot sways: Ctrl+C once, no more motion of that kind today.
- Any contact with the table top or edge (also while the arms rise or lower), the stand knocked over, a finger snags: abort
  that step; no M6 at the table.
- Two exits 3 or 4 in a row in M6: no more VLA today.
- A crash, kill or cable pull during M4-M6: runbook box "crash, kill or cable pull during M4-M6" - do **not** run `86 release`
  yet; the owner decides: table pulled straight back by its far edge, then `86 release`, or the owner's remote. Never
  `86 recover` after a crash in the policy phase.
- Never Ctrl+C an `86 release` that is running: if it looks wrong, the owner's e-stop.
- Never `kill -9`, never close the terminal or the lid while 84 or 86 runs. Ctrl+C twice only if an arm keeps moving towards
  a person.
- Never edit a repo file tomorrow (it locks motion), never the "NEVER" list of runbook page 1 (72 arm, 72 hand --bewegen, 70).

## 5. What committing the unlock accepts

The rulings below are built as proposed defaults ("ruling pending (Finnegan)" in the code). **Committing the unlock =
accepting these rulings** for tomorrow. Not committing it = no motion tomorrow (Parts A and B only, still a useful day).

- **Solo confirmation:** you are the author and present, so the two-operator rule's reason (operators who did not write the
  code) does not hold; check (c) takes `finneganadam` plus the owner's `OWNER` word, still two different people.
- **F1 (feedback lost):** when LowState is older than 0.1 s, 86 keeps sending the last command (nothing moves); if feedback
  returns within 3 s and the arm is where it was sent, it recovers along the planned path, else it holds and you call the
  owner's remote or e-stop.
- **F2 (contact):** a lead above 0.25 rad for 0.26 s makes 86 stop pushing (command = measured angles), hold 0.5 s, then
  recover at half speed (0.15 rad/s); a second contact fault holds for the owner.
- **F3 (invalid target, NaN):** never sent; 86 recovers from the measured pose along the planned path.
- **F4 (second Ctrl+C):** the arms freeze at weight 1; a third Ctrl+C hands them back right there.
- **F5 (internal error, exit overrun, weight step, signal during release):** 86 holds the last command and tells you; you and
  the owner decide (remote/e-stop, or `86 release`).
- **S1:** table steps run only if 87 on today's recorded factory pose says clear (section 3).
- **S2:** at the table only waist 0.6; 0.0/0.15/0.35 make the replay itself hit the stand or basket.
- **S3:** the way out passes the basket rim at 5.06 cm in the sim; accepted, the basket stays on its mark.
- **S4:** 86 moves only along the 87-checked waypoint paths (way in, soft exit, fault recover).
- **S5:** an M0 waist pitch below -0.05 rad (or outside the checked roll/pitch) means no table steps (87 refuses).
- **S6:** `86 recover` refuses after a policy-phase log; then the owner's remote, or table back + `86 release`.
- **r3-m1:** `86 recover` keeps accepting arms up to 0.25 rad off the logged path (not 0.10).
- **r3-m3:** a running release is never cancelled (so `86 recover` never has to finish a killed release's unchecked line);
  if it looks wrong, the owner's e-stop.
- **O1:** the real torso lean is measured on site (the M2-M4 logs and the M4 fence go/no-go), not assumed from the sim.
- **O2-m3:** after a laptop crash in the policy phase, no `86 recover`: the owner's remote, or table back + `86 release`.
- **O2-m4:** from a pose that already touches an object, the robot holds instead of driving home; then remote or Ctrl+C.
- **Fence values:** margins stand 2.0 cm, own body 2.0 cm, table 1.3 cm, basket 0.9 cm; a fence streak (one chunk of held
  messages + 0.5 s, about 2 s at speed 0.5, about 4 s at 0.25) ends in a soft exit; pelvis tilt from the IMU; the 5-action
  chunk blend is on in `run` (never in replay); the replay stand sits 6.7 cm further away and 1.7 cm right (R-F8).

## 6. End of the day

Copy the whole `runs/` folder somewhere safe before you leave (it holds the sheet's data, every 86 log and the factory pose):
```
cp -R runs ~/Desktop/runs_robot_day_2026-11-18
```
Then the photo of the sheet and the videos next to it.
