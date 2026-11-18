# Review: scripts/91_watch.py (fast gate, robot day)

Reviewer: independent; did not author 91, no author reasoning seen. Stage: finished review of one uncommitted file.

## Verdicts
- Implementation fidelity (does 91 do what its docstring says): GO
- Plan correctness (is a read-only side-recorder safe and useful next to 84/86): GO
- Overall: GO. No must-fix items.

## Q1: can 91 change 86's behaviour or safety? No (evidence below)
- No writers: `real_io.install_publisher_guard()` runs at 91:41, before any DDS import that builds a writer. 91 only creates
  readers (real_io readers + one BuiltinDataReader).
- 86 detects other senders by SAMPLES only: real_preflight.py:34 `io.foreign_samples()` = `real_io.active_writers`
  (SampleCounter counts received samples on rt/arm_sdk, rt/lowcmd, hand cmd). 91 sends no samples, so 86 cannot see it.
  91's readers do appear in discovery (DCPSSubscription), but nothing in 86/real_io reads subscriptions.
- Reader QoS: measured in robot-env, a reader with `Qos(History.KeepLast(1))` (what real_io.subscribe_topic uses) gets
  `Reliability.BestEffort, Durability.Volatile`. A best-effort reader sends no ACKs, so it cannot slow down 86's reliable
  rt/arm_sdk writer or the robot's lowstate writer.
- CPU: separate process (no shared GIL). Measured: 91 against the loopback publisher at 500 Hz lowstate + 2 hands =
  9.8 % of one core. On the robot, add rt/odommodestate/sportmodestate if they arrive: estimate about 20 %, acceptable.
- Shared /tmp/cdds.LOG (SDK trace config): every SDK process writes it, not specific to 91; only the loopback test
  helper reads it. No effect on 86.

## Q2: crash or hang that hides the cause? None found
- Teardown: `--seconds 18` run exit 0; Ctrl+C (real SIGINT, SIG_DFL restored) prints the summary, exit 0. No segfault.
  The BuiltinDataReader has no Python listener, so the close_dds/GC segfault path does not apply to it.
- Every row is flushed (91:229-230), so a later crash still leaves the jsonl.
- Ping flags: macOS `-t 1` (timeout s), Linux `-W 1`; regex `time[=<]` does not match "Request timeout" or the Linux
  summary "time 0ms". Loopback run showed NET OK with ping.
- Discovery disposal: verified with two fake rt/arm_sdk publishers on lo0: alive samples have InstanceState 16 (Alive),
  disposal arrives as a DcpsEndpoint with `key` set, `topic_name None`, state 32, so 91:94-96 removes it ("sender gone"
  printed in the run). Disposal after an unclean exit arrives only after the 10 s lease (observed 10 s).
- SportModeState_ on a topic with another type: DDS does not match a different type name, so the reader stays silent
  and BASE prints "no odometry topic (IMU only)", no crash. The robot's real odom topic types are NOT verified here
  (runs/real_probe.json is a loopback probe, no robot data).
- Note for operators testing in a script: a background `&` job in a non-interactive shell ignores SIGINT; that is a shell
  property, not a 91 hang.

## Q3: false or missed alarms (non-blocking, operator notes)
1. WRITERS FAIL counts DISCOVERED writers (91:150-156), while 86 refuses only on SAMPLES. An idle owner tool with an open
   rt/arm_sdk writer, or two robot-internal rt/lowcmd writers, gives "two programs steer the arms" FAIL although nobody
   sends. Safe direction (over-warns). Operator reading: cross-check with ARM / 86's own preflight result.
2. After 86 crashes (no close_dds) and is restarted within ~10 s, the old 86 participant is still "alive" until its
   lease expires: a transient WRITERS FAIL for up to 10 s. A normal 86 exit deletes the participant at once.
3. ALERT is printed on level transitions only (91:236). If WRITERS is already FAIL (point 1), a later real second
   rt/arm_sdk sender raises no new ALERT; the "NEW SENDER rt/arm_sdk" line (91:231-232) still appears.
4. ARM: indices correct (motor_cmd/motor_state 15..28, weight slot 29 = real_send.WEIGHT_SLOT). On fault exits 86 ramps
   out at the current pose (real_safety.ramp_out holds q_hold while weight -> 0), so the factory controller pulls the arm
   away while weight > 0: ARM may WARN "blocked, or another controller" during the 2 s fault ramp-out. The normal release
   ramps out at the factory pose, so no false WARN there.
5. BASE speed threshold 0.05 m/s on odometry of a standing, balancing robot is unverified; it may flap WARN/OK. Only
   checkable on the robot.
6. Participant labels are the first 8 hex chars of the GUID; Cyclone GUIDs start with vendor id 0110, so only 16 bits
   distinguish programs. Collision chance for a handful of participants is negligible; participants in one process
   would share the label (collapsed to one).

## Checks not run
- Nothing against the real robot (topic types of rt/odommodestate / rt/sportmodestate, real rt/lowcmd writer count,
  BASE noise at standstill). These affect only 91's labels, never 86.

## Evidence commands (reviewer, robot-env, lo0)
- QoS: DataReader(..., Qos(Policy.History.KeepLast(1))).get_qos() -> BestEffort, Volatile.
- Discovery: scratchpad obs.py + pub.py (two rt/arm_sdk ChannelPublishers, clean and os._exit) -> states 16 then 32.
- CPU/exit: loopback_lowstate_pub.py --hz 500 --hands + 91 --seconds 18 -> 9.8 % CPU, exit 0; SIGINT run -> summary, exit 0.
- Reviewer-created runs/watch/*.jsonl files were deleted; the two older ones (09:38, 09:39) are not the reviewer's.
