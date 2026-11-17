# Robot day runbook - Unitree G1, Monday 2026-10-19

For Cristina (`acristinaa`) and Saad (`saadsheikh-tech`). Every command is copy-paste. Each step says what you should see
and what to do if you see something else. Run the commands from the repo folder (`cd ~/UnitreeG1-Robot`).

- **Part A** - laptop setup and self-test: Sunday evening after Finnegan's message (push 1), or Monday 08:00 (about 45 min).
- **Part B** - read-only bring-up at the robot: Monday, about 1 h. Nothing in Part B moves the robot.
- **Part C** - motion: arrives with push 2 on Monday morning, only if `86 gate` prints `MOTION: UNLOCKED`.

## Page 1 - rules for the whole day

1. Run only commands from this runbook, exactly as written. Never edit a file in the repo.
2. **The owner and the remote always win.** If the owner says stop, stop.
3. On a FAIL: read that step's "if you see" line, try only what it lists, once. Still FAIL: skip the step and every step
   that depends on it, and write down the time and the last 20 lines of the terminal.
4. Charger on, lid open: **never close the laptop lid** while a command runs. Long commands are printed twice, so the laptop
   does not fall asleep: the line starting with `caffeinate -i` is for macOS, the line starting with `systemd-inhibit` is for
   Ubuntu. Run only the one for your laptop.
5. To stop a running command press **Ctrl+C once**. Twice only if an arm keeps moving towards a person. Never `kill -9`, never
   close the terminal while a command runs.
6. **Every motion step (Part C only):** the robot hangs in the gantry (definition below), the owner holds the remote,
   1.5 m clear around the arms, video on, one operator's hand on Ctrl+C.
7. Who to call: Finnegan, phone `<Finnegan fills in>`. He is travelling; a reply is not guaranteed. `SELFTEST: PASS` and the
   lines printed by each step are enough to go on without him.

### "The robot hangs in the gantry" means exactly this

The robot **stands on its own feet** under the owner's controller, in the arm_sdk mode agreed with the owner (questions Q8/Q10),
and its harness is attached to the gantry with **only a few cm of slack**, so a collapse is caught within a few cm. Type
`HANGING` (Part C asks for it) only in that state - not for a robot lifted off the floor, and not for a harness with long slack.

### NEVER do these

- `python scripts/72_real_check.py arm ...` - it leaves waist roll/pitch without stiffness (kp 0) while it moves an arm.
- `python scripts/72_real_check.py hand --bewegen ...` - it moves the fingers outside every check of this runbook.
- `python scripts/70_real_goto.py ...` - walking is the owner's remote only.
- `kill -9`, closing the terminal or the lid while 84 or 86 runs.
- Editing any file, `git commit`, `git checkout` of another branch, `pip install` by hand.
- Anything that is not in this runbook, even if a README or an old PROGRESS entry shows it.

### Box: "86 release" refused or crashed

Tell the owner to use the remote (the robot hangs in the gantry, so the owner's damping mode is safe). **We do nothing more.**
Write down the time and the last 20 lines of the terminal.

## Part A - laptop setup (Sunday evening or Monday 08:00)

Do this on **both** laptops. The robot cable stays unplugged for all of Part A. Nothing in Part A needs the robot or a login to it.

**Before you start:**
- Bring a blank paper sheet ("the sheet"). You write on it: the robot laptop (below), the policy laptop and its `chunk_ms` (A6),
  the `<IF>` name and the `FACTORY ARMS` line (B0), the Q8 and Q10 answers (B1), hip height, table height and the overlay file
  name (B3), the robot PC's user@IP and the camera number `N` (B3; **never the password**), the layouts you recorded (B4), and
  everything Part C asks for.
- Decide now which laptop is the **robot laptop**: the one that gets the robot cable in Part B and runs 84 (and 86 in Part C).
  Write it on the sheet. Either laptop can be it: both get the robot env in A3.

**A1. Tools.** You need `git`, the GitHub CLI `gh` (version 2.21 or newer) and `uv` (a fast Python installer).
- macOS: `brew install gh` (Homebrew from https://brew.sh if missing).
- Ubuntu: `sudo apt install git curl`. Do **not** take `gh` from Ubuntu itself: on Ubuntu 22.04 that is version 2.4.0, too old
  for the setup script (its model download needs 2.21 or newer). Install `gh` from GitHub's own package source instead (GitHub's
  official instructions, https://github.com/cli/cli/blob/trunk/docs/install_linux.md). Paste the whole block at once; it asks for
  your admin password:
```
(type -p wget >/dev/null || (sudo apt update && sudo apt install wget -y)) \
	&& sudo mkdir -p -m 755 /etc/apt/keyrings \
	&& out=$(mktemp) && wget -nv -O$out https://cli.github.com/packages/githubcli-archive-keyring.gpg \
	&& cat $out | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null \
	&& sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg \
	&& sudo mkdir -p -m 755 /etc/apt/sources.list.d \
	&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
	&& sudo apt update \
	&& sudo apt install gh -y
```
- Both:
```
gh --version
curl -LsSf https://astral.sh/uv/install.sh | sh
gh auth login
```
You should see: `gh --version` prints `gh version 2.` and then a number of 21 or more (for example `gh version 2.102.0`);
`gh auth login` ends with "Logged in as ...". Then **open a new terminal** (so `uv` is found).
If you see an older `gh version` (for example `2.4.0`) on Ubuntu: run the Ubuntu block above once more (it upgrades `gh`).
If you see "command not found: uv" in the new terminal: run the curl line once more and open another new terminal.

**A2. Get the repo.**
```
gh repo clone finneganadam-maker/UnitreeG1-Robot ~/UnitreeG1-Robot
cd ~/UnitreeG1-Robot
```
If the folder already exists: `cd ~/UnitreeG1-Robot && git pull` instead.

**A3. Install everything** (robot env, policy env, model; about 4 GB download on a Mac, about 7 GB on Ubuntu with an NVIDIA card;
20-60 min on home Wi-Fi; re-runnable):
```
caffeinate -i bash scripts/setup_robot_laptop.sh --robot-env --policy-env --release robot-day-2026-10-18
systemd-inhibit --what=sleep:idle:handle-lid-switch bash scripts/setup_robot_laptop.sh --robot-env --policy-env --release robot-day-2026-10-18
```
You should see three lines `SETUP robot-env: OK`, `SETUP release: OK`, `SETUP policy-env: OK`.
If you see `SETUP <block>: FAILED <hint>`: do what the hint says, then run the same command again (finished blocks are quick).
**Which laptops need the policy env** (the model server for Part C): every Apple-silicon Mac and every Ubuntu laptop with an
NVIDIA card (`nvidia-smi` prints a table) - keep `--policy-env` there, because A6 picks the policy laptop from their self-test.
Only a laptop with neither (an Intel Mac, Ubuntu without NVIDIA) skips it: it would run the model on its processor, which is too
slow (2.6 to 6.5 s per answer measured; A6 needs below 1.5 s). On such a laptop run only
`bash scripts/setup_robot_laptop.sh --robot-env`.

**A4. Firewall.** DDS (the robot's data channels) arrives as incoming network packets.
- macOS: check whether the firewall is on:
```
/usr/libexec/ApplicationFirewall/socketfilterfw --getglobalstate
```
  "Firewall is disabled" - nothing to do. "Firewall is enabled" - allow the robot env's Python. The venv's `python` is a link to
  the real program, and we do not know which of the two macOS checks, so allow both (asks for your admin password):
```
PY_LINK=~/g1-robot-day/robot-env/bin/python
PY_REAL="$("$PY_LINK" -c 'import os, sys; print(os.path.realpath(sys.executable))')"
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --add "$PY_LINK"
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --unblockapp "$PY_LINK"
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --add "$PY_REAL"
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --unblockapp "$PY_REAL"
```
  (macOS may also ask the first time 84 runs - click Allow.)
- Ubuntu: `sudo ufw status`. If it says "active": `sudo ufw allow from 192.168.123.0/24`.
- Every laptop that installed the policy env also needs incoming TCP port 8765 (the robot laptop asks the policy laptop for
  actions there; which laptop that is gets decided only in A6, so do this on all of them). macOS, if the firewall is enabled:
```
PY_LINK=~/g1-robot-day/policy-env/bin/python
PY_REAL="$("$PY_LINK" -c 'import os, sys; print(os.path.realpath(sys.executable))')"
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --add "$PY_LINK"
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --unblockapp "$PY_LINK"
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --add "$PY_REAL"
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --unblockapp "$PY_REAL"
```
  Ubuntu, if ufw is active: `sudo ufw allow 8765/tcp`.

**A5. Self-test** (robot cable unplugged):
```
source ~/g1-robot-day/robot-env.sh
caffeinate -i python scripts/89_robot_day_selftest.py
systemd-inhibit --what=sleep:idle:handle-lid-switch python scripts/89_robot_day_selftest.py
```
On a laptop without the policy env: `python scripts/89_robot_day_selftest.py --skip-policy` (short, no prefix needed).
You should see one `CHECK <name>: PASS` line per check and at the end `SELFTEST: PASS`. The policy check loads the model and takes
1-5 min. Also fine: `CHECK policy: SKIP --skip-policy` on a laptop run with `--skip-policy`, and a `CHECK motion: INFO ...` line
(newer versions print it; it only reports whether Part C is unlocked and never fails).
If you see:
- `CHECK dds: FAIL a 192.168.123.x address exists` - unplug the robot cable (or turn off that network adapter), run again.
- `command not found: python`, or `ModuleNotFoundError` (no `CHECK` line at all) - you forgot the `source` line; run both lines
  again.
- `CHECK versions: FAIL` - the wrong environment is active; run both lines again.
- `CHECK policy: FAIL backbone not in the prefix` - run the setup's policy block again (first line macOS, second line Ubuntu, as
  in A3), then A5 again:
```
caffeinate -i bash scripts/setup_robot_laptop.sh --policy-env
systemd-inhibit --what=sleep:idle:handle-lid-switch bash scripts/setup_robot_laptop.sh --policy-env
```
- `CHECK policy: FAIL ... missing` - run A3 again with `--policy-env --release robot-day-2026-10-18`, or use `--skip-policy` on
  this laptop.
- any other FAIL - run 89 once more; still FAIL: send the `runs/robot_day_selftest_*.json` file and a photo of the screen to
  Finnegan and go on with the other laptop. For a `CHECK policy: FAIL 81 --self-test ...` line also send the
  `runs/robot_day_selftest_policy_<device>.log` file that the line names (it holds the model's full output).
If 89 is missing ("No such file"), it arrives with a later push: run `git pull` on Monday 08:00 and do A5 then.

**A6. Decide where the policy server runs (ruling R11): the "policy laptop".**
1. Candidates: first the robot laptop, if it is an Apple-silicon Mac (it runs the server itself). Then the other laptop, if its
   `CHECK policy` line passed with `mps` or `cuda` (a pass with `cpu` does not count).
2. Look at `chunk_ms` in each candidate's `CHECK policy` line: it must be below 1500 on every candidate, the Apple-silicon robot
   laptop too (at speed 0.5 the robot asks for a new chunk every 25 actions = 1.7 s). `chunk_ms` is the model's very first
   answer after loading, its slowest one (Finnegan's Apple-silicon Mac: 1318 ms first, 207 ms per answer after that). If it is
   1500 or more: run 89 once more on that laptop (A5) and look again.
3. The policy laptop is the first candidate below 1500. If there is none: write that on the sheet and tell Finnegan; Part C then
   has no policy server (no M5, no M6).
Write the policy laptop and its `chunk_ms` on the sheet.

## Monday 08:00 - before going to the robot

```
cd ~/UnitreeG1-Robot
git pull
source ~/g1-robot-day/robot-env.sh
caffeinate -i python scripts/89_robot_day_selftest.py
systemd-inhibit --what=sleep:idle:handle-lid-switch python scripts/89_robot_day_selftest.py
python scripts/86_real_run.py gate
```
On the laptop without the policy env, run 89 with `--skip-policy` (otherwise its policy check fails).
You should see `SELFTEST: PASS`. Then `86 gate`:
- `MOTION: UNLOCKED` - Part C is allowed today (after Part B).
- `MOTION: LOCKED` or "No such file" - today is Parts A and B only. That is a complete, useful day (real data).
If Finnegan's message says push 2 slipped: `git pull` again at 08:30, and once more before Part C if it landed after 08:30.

## Part B - read-only bring-up at the robot (Monday, about 1 h; nothing moves)

The owner places the robot with their own remote. You only listen. Do all of Part B on the **robot laptop**, in **one
terminal**; if you open a new one, first run `cd ~/UnitreeG1-Robot` and `source ~/g1-robot-day/robot-env.sh`. In every command,
replace `<IF>` with the name of the cable adapter from B0 (for example `en5` on macOS, `enx00e04c...` on Ubuntu).
- Every 84 run also prints `STEP <name>: NOT RUN not selected (--steps)` for the steps you did not ask for, then
  `OWN WRITERS: 0 (must be 0)` and `REPORT runs/real_bringup_<time>.json` - that is normal. `OWN WRITERS` not 0: stop Part B and
  tell Finnegan.
- If 84 ends with `Fatal Python error: Segmentation fault`, the STEP lines printed above it are valid, but no report was
  written. Run 84 again with the same flags and keep the log for the build team (copy the whole terminal text of both runs into
  a note). The hand-state step listens 1 s before it reports the rate; the robot's hand service should show about 100 Hz, and
  below 50 Hz is only a warning.
- If a FAIL line suggests a `72 ...` command: do not run it unless this runbook lists it (only `72_real_check.py kamera --suchen`
  in B3).

**B0. Network.** Plug the robot cable into the laptop. Give the cable adapter the fixed address `192.168.123.222`, mask
`255.255.255.0`:
- macOS: System Settings > Network > the USB/Thunderbolt LAN adapter > Details > TCP/IP > Configure IPv4 "Manually".
- Ubuntu: Settings > Network > the wired adapter > IPv4 > Manual (address 192.168.123.222, netmask 255.255.255.0).
```
bash scripts/setup_robot_laptop.sh --show-ip
ping -c 3 192.168.123.164
```
You should see a line like `  en5: 192.168.123.222` (Ubuntu: `  enx00e04c...: 192.168.123.222/24`) and three ping replies
(`64 bytes from 192.168.123.164 ...`). `<IF>` is the name in front of the colon, **without the colon** (`en5`, not `en5:`); write
it on the sheet. If not: check the cable and the address once, then ask the owner which port to use. If the address is right but
ping still gets no reply: run the 84 line below anyway - its probe step decides (84 counts a silent ping only as a warning).
```
source ~/g1-robot-day/robot-env.sh
python scripts/84_real_bringup.py --iface <IF> --steps net,probe,joints
```
You should see `STEP net: PASS`, `STEP probe: PASS`, `STEP joints: PASS` and a line `FACTORY ARMS: ...` - copy that line onto the
sheet (Part C needs it). No factory pose is recorded here; Part C records it in the mode Part C uses.
If you see one of these, do what is listed once and run the 84 line again; if it is still there, **abort Part B** (tell the owner,
note the time):
- `no fresh rt/lowstate` (no LowState): check the cable, the address and that the robot is on.
- `rt/lowstate only ... Hz` (LowState below 200 Hz): close other programs on the laptop.
- `samples on rt/arm_sdk` (someone else is commanding the arms): ask the owner to stop it.
- `outside the joint limits` (a joint out of range): no retry - abort Part B at once and show the owner.

**B1. Owner questions** (Q1-Q12; 84 asks them and stores the answers):
```
python scripts/84_real_bringup.py --iface <IF> --steps questions
```
When 84 asks Q2, also ask which `/dev/videoN` is the D455's colour stream and add it to the Q2 answer (if the owner does not know,
B3 finds it). When 84 asks Q3, also ask and add to the answer: the head with the D455 is motorized - how is it set (app, remote,
program), can it be set to straight ahead and 50 deg down and left there all day, and what else can move it (head tracking,
behaviours, apps, BotBrain)? Q5 asks only for the user and IP: **never type the password into 84 or onto the sheet** (84 saves
every answer in `runs/`, and `runs/` is copied at the end of the day). If 84 has no questions step, ask from the paper list (the
same words 84 uses): Q1 Backpack PC: which computer is in the backpack, and may we use it? Q2 D455: which computer is it plugged
into, USB 3 or 2, is another program using it, and which `/dev/videoN` is its colour stream? Q3 Head pitch: is the head camera
mount tilted, and by how many degrees? (Also ask the head questions above.) Q4 BrainCo hands: does `brainco_hand_service` run on
PC2 (`rt/brainco/*`)? Q5 SSH/IP: user and IP of the robot PC with the camera and hands (default `unitree@192.168.123.164`)? Q6
Waist: are waist roll and pitch locked on this robot? Q7 Built-in arm service: does a built-in service already command the arms?
Q8 arm_sdk: is `rt/arm_sdk` allowed on this firmware, and in which robot mode (remote combo)? Q9 Robot time: how long may the
robot run today, and when are the breaks? Q10 Firmware: which firmware version runs on the robot? Q11 arm_sdk stops: what does the
robot do when `rt/arm_sdk` messages stop? Q12 Owner motion: may we record while the owner moves the arms with their own tools?
**Q8 and Q10 must be answered before Part C.**

**B2. Hand state:**
```
python scripts/84_real_bringup.py --iface <IF> --steps hand-state
```
You should see `STEP hand-state: PASS both hands report (0 = open, 1 = closed)` (the six values per hand are in the
`runs/real_bringup_<time>.json` report). If FAIL: ask the owner whether `brainco_hand_service` (its program is called
`brainco_hand_server`) runs on the robot PC (Q4); if the owner starts it, run B2 once more. Still FAIL or not started: skip B2.

**B3. Table, head camera, robot PC login and camera.** Left and right are always the robot's left and right.
- **Robot state first.** Ask the owner to put the robot (with the remote) in the mode it will stand in at the table in Part C
  (the arm_sdk mode from Q8) and to leave that mode and the stand height alone for the rest of the day: the hip height depends on
  the mode. If the owner keeps another mode for now, measure now and once more after the mode switch in M0 (Part C checks it).
  Then mark both feet on the floor with tape, at the toe and the heel of each foot: Part C checks the robot still stands there.
- **Hip height** = floor to the hip pitch joint axis. The hip pitch joint is the top-most leg joint; its axis runs left-right, so
  seen from the side it is the centre of the round motor face at the top of the leg. The hip roll joint sits 3 cm lower and 5 cm
  further out, with its axis pointing forward - do not measure to it. Not sure which motor it is: ask the owner to point at the
  hip pitch motor. Hold a string with a small weight at the centre of that motor (do not push the robot), tape the floor point
  under it (the **hip point**), and measure the hip height along the string.
  It should be near 0.65 m (simulator: 0.648 m). Below 0.60 m or above 0.72 m: you probably measured to another point (the old
  mistake, the pelvis centre, is 10 cm higher) - measure again. Still outside: ask the owner to point at the hip pitch motor,
  measure there, and write the number with "owner checked" on the sheet. Still above 0.72 m: a straight-legged G1 cannot be
  that tall, so it is a mis-measurement - never set the table above 0.81 m, and skip M4-M6 at the table.
- **Table top height** = 0.74 m + (hip height - 0.648 m); for a hip height of 0.60-0.72 m that is 0.69-0.81 m. Write both
  numbers on the sheet. (In the simulator the table top is 9 cm above the hip axis.) If the owner changes the mode or the stand
  height later in the day: measure the hip height again and re-set the table.
- **Table position** (the training layout): the table's front edge straight across in front of the robot, **24.5 cm** in front
  of the hip axis - measure along the floor from the hip point straight forward to the point under the table's front edge (hold
  the string at the front edge too) - and the table centred left-right on the robot (in training the table was 90 cm wide and
  65 cm deep). Mark its two front corners on the floor with tape (Part C may add a second mark). If the 24.5 cm turns out wrong
  later in B3, move the table and re-tape the marks; after B3 the marks are fixed.
- On the table, measured back from the front edge and sideways from the robot's centre line: the stand's centre (with the
  volleyball on it) 14.4 cm back and 24.1 cm to the robot's **left**; the basket (if there is one) with its centre 14.4 cm back
  and 24.1 cm to the **right**.
- **Head camera.** The head with the D455 is motorized and the robot can move it: **never move it by hand** (pushing a powered
  servo can damage it, or the head moves back on its own). Ask the owner to set it with the head's own control (Q3) to straight
  ahead (looking along the robot's centre line, no turn left or right) and **50.0 deg down** (in the simulator: 48.7 deg head
  tilt + 2.3 deg mount - 1.0 deg pelvis lean). You only check: a phone held lightly flat against the D455's top face reads
  49 to 51 deg in its level app, and the overlay in the camera step below shows the table centred left-right. Outside 49-51:
  the owner corrects it with the head's control, then check again.
- **The head stays put after B3.** Once the camera step below has passed, ask the owner to turn off everything that can move the
  head (head tracking, behaviours, apps, BotBrain) and not to touch the head's control for the rest of the day. If the head moves
  anyway: the owner sets it again as above, then run the 84 camera line again and compare its overlay with the B3 one. Part C
  repeats this check before M4, before M5 and after any mode change or reboot.

Robot PC login (only on site at SAP, from the owner): ask the owner for the robot PC's login - user and password - and confirm
its IP (Q5; the computer the D455 is on, Q2; usually `unitree@192.168.123.164`). Log in once by hand first, because 84 waits only
30 s for the login:
```
ssh -o ConnectTimeout=5 unitree@192.168.123.164 echo ok
```
The first time, ssh asks `Are you sure you want to continue connecting (yes/no/[fingerprint])?` - type `yes` and Enter. Then it
asks `unitree@192.168.123.164's password:` - type the password (nothing appears while you type) and Enter. You should see `ok`.
If you see `Permission denied`: ask the owner for the password again and try once more. `Connection timed out` or `No route to
host`: wrong IP or that computer is off - ask the owner. Still no `ok`: skip B3 and B4.
If the owner gave another user or IP, use that user@IP in this line and in every `--robot-pc` below instead of
`unitree@192.168.123.164`, and write it on the sheet. The 72 and 84 camera commands below log in again each time and ask for the
password again: type it **within 30 s**, or 84 gives up with `camera did not start ...`.

Camera number `N` (the D455's `/dev/videoN`): if the owner named it in Q2, use it. Otherwise list the robot PC's cameras (this only
reads; nothing moves; it asks for the password; it always uses `unitree@192.168.123.164`):
```
python scripts/72_real_check.py kamera --suchen
```
It prints one line per video device, for example `/dev/video4: YUYV` (the closing hint is in German). `N` is the number of the
device that shows `YUYV` (the colour stream; `Z16` is depth, `GREY` is infrared, nothing after the colon is a data-only device).
Several with `YUYV`: try the lowest first. Also look at:
- the first lines: the D455 is the one with `8086:0b5c` (other `8086` lines are other Intel devices).
  `keine Intel-RealSense am Bordrechner` means the D455 is not on this computer: ask the owner which computer it is on (Q2), log
  in there as above, and find `N` by trying (last point).
- no `/dev/video` lines at all: a login problem (ssh's error is printed above) - redo the ssh line above, then run 72 again.
  Check this first: after a failed login the line below is empty too.
- the last line before the German hint: it must start with `opencv`. Anything else (after a working login): the robot PC has no
  OpenCV - tell the owner; skip B3 and B4.
- nothing after the colon on every line, or another user@IP than `unitree@192.168.123.164`: try N = 0, 1, 2, ... up to 7 in the
  camera command below; the right one gives `STEP camera: PASS`.
```
python scripts/84_real_bringup.py --iface <IF> --steps camera --robot-pc unitree@192.168.123.164 --camera-device N
```
Replace `N` with the camera number and write it on the sheet. You should see `STEP camera: PASS framing OK; check the overlay by
eye` (or `STEP camera: PASS` followed by a warning about fps or colours instead - also fine) and a line `  saved runs/real_camera_overlay_<time>.png`. Open that file with
the name from the line (macOS `open runs/real_camera_overlay_<time>.png`, Ubuntu `xdg-open runs/real_camera_overlay_<time>.png`).
It is the camera picture laid 50/50 over the training picture `docs/robot_day/reference_head_frame.png`: the table's front edge
and the stand must lie on top of each other, the table centred left-right. Write the overlay file name on the sheet (Part C
compares against it). If they do not line up: measure the 24.5 cm once more (wrong: move the table and re-tape the marks), and
check the head once more with the phone (49-51 deg) and straight ahead; if it is off, the owner corrects it with the head's
control (never by hand). Then run the 84 line again. Still not lined up: **do not move the table off its tape marks and do not
ask for the head to be tilted beyond 49-51 deg** (Part C's table check assumes this exact position) - write on the sheet what is
off (which way, roughly how much in the picture) and go on.
If you see:
- `wrong number or camera busy`: try the next `YUYV` number (or the next N). If every number gives it, the camera is busy: ask
  the owner to stop the other camera program (Q2), run again.
- `camera did not start` with `Permission denied` in it, or after you typed nothing: a login problem - run again and type the
  password within 30 s.
- `it must run at 1280 x 800`: this is not the colour stream - try the next number. If the `YUYV` device gives it, ask the owner
  whether the D455 is on USB 3 (Q2) - on USB 2 the camera may not offer this size; the owner moves it to a USB 3 port, run again.

**B4. Recordings** (3-5 table layouts, robot still, nobody moves it). A layout is one placement of the ball on its stand (and the
basket) on the table. `layout1` is the B3 placement. For each further layout move the stand with the ball a few cm (at most 10 cm,
the range the model was trained on) and the basket at most 8 cm; the table stays on its tape marks. Write each layout on the
sheet (for example "layout2: stand 5 cm back").
```
caffeinate -i python scripts/84_real_bringup.py --iface <IF> --steps record --robot-pc unitree@192.168.123.164 --camera-device N --seconds 20 --label layout1
systemd-inhibit --what=sleep:idle:handle-lid-switch python scripts/84_real_bringup.py --iface <IF> --steps record --robot-pc unitree@192.168.123.164 --camera-device N --seconds 20 --label layout1
```
`N` and the user@IP are the ones from B3; each line asks for the password (within 30 s). You should see
`STEP record: PASS 600 frames -> runs/real_observe/<time>_layout1.npz` (20 s at 30 frames per second). If you see:
- `foreign rt/arm_sdk samples while recording`: something commanded the arms (the recording is saved anyway). Stop recording,
  ask the owner what moved the arms, and record again only after the owner has stopped it.
- `camera stopped during the recording` or `rt/lowstate stopped during the recording`: run the same line once more.
Change `layout1` to `layout2`, ... for each layout. If the owner offers to move the arms with their own tool (Q12): both of you
stand **at least 1.5 m from the arms**, then use `--label owner_motion` and write down which tool and mode the owner used.
After the last layout put the stand, ball and basket back to the B3 placement.

**End of the day:** copy the whole `runs/` folder to the shared folder Finnegan names in his message (if he named none, leave it
on the robot laptop and delete nothing) and send Finnegan the file names.

## Part C - motion (only if UNLOCKED)

Do Part C only if **all** of these hold: `86 gate` printed `MOTION: UNLOCKED` this morning, Part B is done, the owner answered
Q8 and Q10, and the owner has switched the robot to the arm_sdk mode you agreed on. Every step below is a motion step: page 1
rule 6 applies every time (robot hangs in the gantry, owner holds the remote, 1.5 m clear around the arms, video on, one hand on
Ctrl+C). Before it moves anything, 86 asks for both your GitHub names and for the word `HANGING`.

**How an 86 run ends - say this out loud before M1, so nobody reaches in:**
- **Soft exit** (normal end, one Ctrl+C, exit code 3): the arms first rise back to the ready pose, **the hands open there** (a
  held ball drops onto the table), then the arms lower slowly. This takes a while; let it finish.
- **Fault exit** (exit code 4): the arms lower from wherever they are within about 2 s; the fingers stay as they are.
- The last two lines are always `86 <MODE>: DONE | REFUSED ... | SOFT EXIT ... | FAULT EXIT ...` and `LOG <file>`. Copy both
  onto the paper sheet with the time.
- If something goes wrong: the boxes at the end of Part C (hand tool, crash at the table, after any crash).

**M0. Record the factory pose** (before the first 86 command of the day). The owner switches to the agreed arm_sdk mode, stops
their own arm tool if they used it in B4, and leaves the arms where the controller holds them. Then:
```
source ~/g1-robot-day/robot-env.sh
python scripts/84_real_bringup.py --iface <IF> --steps joints --record-factory-pose --mode-note "<owner's Q8 answer>"
```
You should see `STEP joints: PASS` with "factory pose recorded", a `FACTORY ARMS: ...` line and a `PART B ARMS: gap ... rad`
line. Copy the FACTORY ARMS line onto the sheet. If you see:
- `WARNING: differs from the Part B reading by more than 0.10 rad` and the owner did **not** change the mode: the owner re-sets
  the robot (or confirms their tool ended at weight 0), then run M0 again. If the owner did change the mode, go on.
- `another program steers the arms right now`: the owner stops their tool, then run M0 again.
- `a different factory pose was recorded today`: only if the owner confirms the arms are in the controller's hold pose (for
  example after a mode change), run once:
```
python scripts/84_real_bringup.py --iface <IF> --steps joints --record-factory-pose --overwrite-factory-pose --mode-note "<owner's Q8 answer>"
```
  Otherwise ask the owner, and stop Part C.
- `our last 86 run may still hold the arms`: run the M1 command (`86 release`) first, then M0 again. Release refused: page 1 box.

Repeat M0 whenever the owner changes the robot mode later in the day. After this first mode switch and after every later mode
change or reboot, also do the robot and head check (M4) before the next M4, M5 or M6 line: a mode change may change the stand
height or move the head.

**M0 lookup - is the table safe for the arms?** The arms move on straight lines between the factory pose and the ready pose.
Whether those lines (and every way out of a demo pose) clear the table edge was computed in the simulator (script 87). Apply it
to the FACTORY ARMS line now and write the result on the sheet:
1. **Bounds line:** if any shoulder roll, shoulder yaw, the largest wrist or the waist yaw is more than 0.15 from 0, a shoulder
   pitch is outside -0.1 .. 0.3, or an elbow is outside 0.0 .. 1.6: **skip M4-M6 at the table.** Stop the lookup here.
2. **Elbow row:** take the larger of the two elbow values and round it **up** to the next 0.1 (0.93 -> 1.0; 0.90 stays 0.9).
   Find that row below. "table position" says where the table stands for M4-M6: at the Part B tape mark, moved back N cm (put a
   second tape mark there), or skip.
3. **Waist column:** the columns are the four waist settings 87 checked: hold = `--waist-hold`, 0.15 = `--waist-max 0.15`,
   0.35 = `--waist-max 0.35`, 0.6 = `--waist-max 0.6`. At the table use **exactly one** of these four, never more than the
   limit reached in M3, and read that column. If it says "no", a lower checked setting whose column says "yes" may be used
   (if M3 did not run that setting, one in-air replay with it comes first, M3). Otherwise skip the table steps.

| elbow | table position | M4 hold | M4 0.15 | M4 0.35 | M4 0.6 | M6 hold | M6 0.15 | M6 0.35 | M6 0.6 |
|---|---|---|---|---|---|---|---|---|---|
| 0.0 | skip | no | no | no | no | no | no | no | no |
| 0.1 | skip | no | no | no | no | no | no | no | no |
| 0.2 | skip | no | no | no | no | no | no | no | no |
| 0.3 | skip | no | no | no | no | no | no | no | no |
| 0.4 | skip | no | no | no | no | no | no | no | no |
| 0.5 | skip | no | no | no | no | no | no | no | no |
| 0.6 | skip | no | no | no | no | no | no | no | no |
| 0.7 | skip | no | no | no | no | no | no | no | no |
| 0.8 | skip | no | no | no | no | no | no | no | no |
| 0.9 | skip | no | no | no | no | no | no | no | no |
| 1.0 | skip | no | no | no | no | no | no | no | no |
| 1.1 | skip | no | no | no | no | no | no | no | no |
| 1.2 | skip | no | no | no | no | no | no | no | no |
| 1.3 | skip | no | no | no | no | no | no | no | no |
| 1.4 | skip | no | no | no | no | no | no | no | no |
| 1.5 | skip | no | no | no | no | no | no | no | no |
| 1.6 | skip | no | no | no | no | no | no | no | no |

**What this means today: no factory pose is cleared for the table.** Every row says skip, in every waist column (87 report
`runs/arm_path_clearance.json`, 2026-10-27, upper body leaning forward 0.126 rad as in the sim). Raising the arms from any
hanging pose to the ready pose sweeps a hand through the basket or the table, and not every way out of a demo pose stays 5 cm clear.
So Part C ends after M3 at the table: skip M4, and run M5 and M6 only with the table moved away.

Checked and still not enough (87 `--arms-up-first`, `.planning/real-robot/table-clearance-variant.md`): raising the arms with the
table at least 30 cm farther back and then pushing the table in with the arms held at the ready pose is clear (at least 9.9
cm). But every way out that ends in the factory pose - the soft exit's second leg, the fault exit, and `86 release` after a
crash - still goes 5 to 16 cm into the table or basket. Do not try this layout on the robot day: it is a decision for Finnegan
and the owner, not a cleared step.

**M1. Release** (proves the hand-back moves nothing):
```
python scripts/86_real_run.py release --iface <IF>
```
You should see nothing visible move and `86 RELEASE: DONE`. **Abort** (no more motion today, owner's remote if needed): any
jerk or swing.

**M2. Ready pose** (table moved away, arms in the air):
```
python scripts/86_real_run.py ready --iface <IF> --hold 5
```
You should see the arms rise slowly to the ready pose, hold 5 s with `hold: lead ... rad` lines, lower again, then
`86 READY: DONE`. Compare the pose with the ready pose in `docs/robot_day/volleyball_natural_phases.png` (open it before you
start). **Abort:** a jerk while the arms start or stop; the pose is mirrored or a joint moves the wrong way; a lead above 0.10
rad during the hold; the robot sways.

**M3. Slow replay of the demo, in the air** (table moved away). In a **second terminal** on the robot laptop, start the replay
server and leave it running (it only plays back the recorded demo):
```
cd ~/UnitreeG1-Robot
source ~/g1-robot-day/robot-env.sh
python scripts/81_smolvla_server.py --replay tests/data/replay_demo.npz --port 8766
```
You should see `LISTENING 8766`. Then in the first terminal, one line after the other, each only after the one before ended
with `86 REPLAY: DONE` (about 97 s at speed 0.25, 49 s at 0.5):
1. speed 0.25, waist up to 0.15:
```
caffeinate -i python scripts/86_real_run.py replay --iface <IF> --server 127.0.0.1:8766 --episode replay_demo --no-camera --speed 0.25 --waist-max 0.15
systemd-inhibit --what=sleep:idle:handle-lid-switch python scripts/86_real_run.py replay --iface <IF> --server 127.0.0.1:8766 --episode replay_demo --no-camera --speed 0.25 --waist-max 0.15
```
2. speed 0.5, waist up to 0.15:
```
caffeinate -i python scripts/86_real_run.py replay --iface <IF> --server 127.0.0.1:8766 --episode replay_demo --no-camera --speed 0.5 --waist-max 0.15
systemd-inhibit --what=sleep:idle:handle-lid-switch python scripts/86_real_run.py replay --iface <IF> --server 127.0.0.1:8766 --episode replay_demo --no-camera --speed 0.5 --waist-max 0.15
```
3. speed 0.5, waist up to 0.35:
```
caffeinate -i python scripts/86_real_run.py replay --iface <IF> --server 127.0.0.1:8766 --episode replay_demo --no-camera --speed 0.5 --waist-max 0.35
systemd-inhibit --what=sleep:idle:handle-lid-switch python scripts/86_real_run.py replay --iface <IF> --server 127.0.0.1:8766 --episode replay_demo --no-camera --speed 0.5 --waist-max 0.35
```
4. only if the owner says yes - speed 0.5, waist up to 0.6:
```
caffeinate -i python scripts/86_real_run.py replay --iface <IF> --server 127.0.0.1:8766 --episode replay_demo --no-camera --speed 0.5 --waist-max 0.6
systemd-inhibit --what=sleep:idle:handle-lid-switch python scripts/86_real_run.py replay --iface <IF> --server 127.0.0.1:8766 --episode replay_demo --no-camera --speed 0.5 --waist-max 0.6
```
If the robot sways when the waist turns already in line 1: stop there and use the waist-hold line instead (waist stays still):
```
caffeinate -i python scripts/86_real_run.py replay --iface <IF> --server 127.0.0.1:8766 --episode replay_demo --no-camera --speed 0.5 --waist-hold
systemd-inhibit --what=sleep:idle:handle-lid-switch python scripts/86_real_run.py replay --iface <IF> --server 127.0.0.1:8766 --episode replay_demo --no-camera --speed 0.5 --waist-hold
```
**Abort** (stay at the last clean line): exit 3 or 4; the motion differs from the demo pictures; a hand heads into the body;
the robot sways at a waist step. Write the waist setting of the last clean line on the sheet (hold, 0.15, 0.35 or 0.6). **Never
use any other waist value at the table**: 87 checked only these four.

**M4. The same replay at the table** (real data). **Precondition:** the sheet says "M4 at the table: yes" in the column of the
waist setting reached in M3 (or of a lower checked setting that says yes). "no", "skip", or the bounds line
-> skip M4 and M6 at the table (today every row says skip, see the M0 lookup).
**Robot and head check** (before M4, before M5, and after any mode change or reboot). Put the table on the **Part B** tape marks
with stand and ball as in B3; no 86 command runs (the same state as in B3).
1. Robot: both feet on their B3 tape marks, and the hip point still 24.5 cm (within 1 cm) behind the table's front edge (string
   from the hip pitch motor, as in B3). Off: the owner puts the robot back on its marks with the remote (never push it yourself),
   then check again; still off: skip M4-M6 at the table. Then measure the hip height again: more than 1 cm away from the number
   on the sheet (for example after a mode change): re-set the table height with the B3 formula and write both new numbers on the
   sheet.
2. Head: run the B3 camera line again and open the new overlay next to the B3 one (file name on the sheet):
```
python scripts/84_real_bringup.py --iface <IF> --steps camera --robot-pc unitree@192.168.123.164 --camera-device N
```
   The table's front edge and the stand must sit at the same place in both pictures. Different: the owner sets the head again
   with its own control (B3; never by hand), then run the line again. Still different: skip M4-M6 (the policy would see a view it
   was not trained on) and write it on the sheet.

Then put the table at the tape mark from the lookup (if it says "back N cm", the hip point is now 24.5 cm + N cm behind the
front edge), the ball on its light stand. Replace
`<WAIST>` with exactly that setting (`--waist-hold`, `--waist-max 0.15`, `--waist-max 0.35` or `--waist-max 0.6`) and `N` with
the camera number from B3:
```
caffeinate -i python scripts/86_real_run.py replay --iface <IF> --server 127.0.0.1:8766 --episode replay_demo --robot-pc unitree@192.168.123.164 --camera-device N --speed 0.5 <WAIST>
systemd-inhibit --what=sleep:idle:handle-lid-switch python scripts/86_real_run.py replay --iface <IF> --server 127.0.0.1:8766 --episode replay_demo --robot-pc unitree@192.168.123.164 --camera-device N --speed 0.5 <WAIST>
```
**Abort:** any contact with the table top or edge, also while the arms rise or lower; the stand is knocked over; a finger
snags. A brush that moves the stand less than 2 cm is written down, not an abort. After M4, stop the replay server (Ctrl+C in
the second terminal).

**M5. The VLA watches, nothing it says is sent** (`shadow`). First the robot and head check from M4 once more (also if M4 was
skipped; move the table away again afterwards if M5 runs without it). Start the policy server on the R11 laptop (Reference
section) and wait for `LISTENING 8765`. `<POLICY-HOST>` = `127.0.0.1` if the robot laptop runs the server itself, else the other
laptop's Wi-Fi address (macOS `ipconfig getifaddr en0`, Ubuntu `hostname -I`, first number). Use the same `<WAIST>` as in M4 (or
any setting reached in M3 if the table is away):
```
python scripts/86_real_run.py shadow --iface <IF> --server <POLICY-HOST>:8765 --robot-pc unitree@192.168.123.164 --camera-device N --max-seconds 30 <WAIST>
```
You should see `shadow: X of Y policy ticks envelope-clipped (Z %; ...)` and `86 SHADOW: DONE`. **Abort** (no M6): Z above 5 %,
or `NaN` anywhere in the output.

**M6. The VLA moves the arms** (`run`). **Precondition at the table:** the sheet says "M6 at the table: yes" in the column of the
waist setting used in M4; otherwise M6 at the table is skipped. M6 with the table moved away is optional (nothing to hit; it
only shows whether the policy moves sensibly on real camera frames). At least 3 filmed attempts, 60 s each at most; first speed
0.25, then 0.5:
```
caffeinate -i python scripts/86_real_run.py run --iface <IF> --server <POLICY-HOST>:8765 --robot-pc unitree@192.168.123.164 --camera-device N --speed 0.25 <WAIST>
systemd-inhibit --what=sleep:idle:handle-lid-switch python scripts/86_real_run.py run --iface <IF> --server <POLICY-HOST>:8765 --robot-pc unitree@192.168.123.164 --camera-device N --speed 0.25 <WAIST>
caffeinate -i python scripts/86_real_run.py run --iface <IF> --server <POLICY-HOST>:8765 --robot-pc unitree@192.168.123.164 --camera-device N --speed 0.5 <WAIST>
systemd-inhibit --what=sleep:idle:handle-lid-switch python scripts/86_real_run.py run --iface <IF> --server <POLICY-HOST>:8765 --robot-pc unitree@192.168.123.164 --camera-device N --speed 0.5 <WAIST>
```
**Abort:** two exits 3 or 4 in a row -> no more VLA today. Expect misses: the model has only seen the simulator. A clean exit
and the recorded data are a success.

### Box: 86 prints `foreign_hand_cmd`

Another program sends hand commands. Ask the owner to stop their hand tool and run the same command again. If it cannot be
stopped, run the same command with `--no-hands` added.

### Box: crash, kill or cable pull during M4-M6 (table in place)

86 did not print its final `86 <MODE>:` line, so our last command may still hold the arms over the table. **Do not run
`86 release` yet.** The owner decides whether the table (with stand and basket) can be pulled straight back, away from the robot.
- Yes: one person pulls the table back by its **far edge** (never reach between the arms and the table), then `86 release`.
- No: the owner's remote (box "86 release refused or crashed" on page 1).
- After an exit 4 the arms are already handed back (`86 release` should move nothing); after an exit 3 they are already down.
  Either way, write down whether an arm touched the table on the way down.

### After any crash, cable pull or abort

If it happened during M4-M6: the box above first. Then the M1 command (`86 release`), then, if the fingers are closed:
```
python scripts/86_real_run.py hands-open --iface <IF>
```
If release is refused or crashes: box "86 release refused or crashed" on page 1. We do nothing more.

## Reference

- Policy server for Part C, on the R11 laptop (first line macOS with `--device mps`, second line Ubuntu + NVIDIA with `--device cuda`):
```
source ~/g1-robot-day/policy-env.sh
caffeinate -i python scripts/81_smolvla_server.py --checkpoint ~/g1-robot-day/smolvla-runs/run1/checkpoints/040000/pretrained_model --device mps --host 0.0.0.0
systemd-inhibit --what=sleep:idle:handle-lid-switch python scripts/81_smolvla_server.py --checkpoint ~/g1-robot-day/smolvla-runs/run1/checkpoints/040000/pretrained_model --device cuda --host 0.0.0.0
```
  You should see `LISTENING 8765`. `tests/data/replay_demo.npz` (the slow replay) has blank images on purpose: the replay server
  only uses images for diagnostics.
