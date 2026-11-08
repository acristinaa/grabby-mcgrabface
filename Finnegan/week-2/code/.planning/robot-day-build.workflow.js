export const meta = {
  name: 'robot-day-build',
  description: 'Plan rev 6 for Monday 10-19 (VLA on the real G1, operated by Cristina+Saad), plan review loop, 5 parallel build tracks with per-track review/fix, sim integration, runbook dry walk',
  phases: [
    { title: 'Plan', detail: 'planner rev 6 + reviewer loop until GO' },
    { title: 'Build', detail: 'tracks A, B1, B2, C, E: build -> review -> fix' },
    { title: 'Integrate', detail: 'full tests + sim-to-sim run, fixes' },
    { title: 'Dry walk', detail: 'fresh agent follows the runbook against the sim' },
    { title: 'Final gate', detail: 'judge over everything' },
    { title: 'Testing guide', detail: 'short guide that explains every new test for Finnegan' },
  ],
}

const REPO = '~/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot'
const COMMON = `Repo: \`${REPO}\`. Your shell starts in a DIFFERENT clone (an Emdash worktree): first \`cd\` into the Repo path above, resolve every relative path below against it, and never read or write project files in the starting directory. Plan: \`.planning/real-robot/PLAN.md\`. Reviews: \`.planning/reviews/\`. Status file: \`.planning/STATUS.md\` (append one dated line when you finish: who you are, result).
Hard rules: never commit or push (the user does that). Never touch a real robot (none is connected). Third-party code only under ~/dev/g1-ext. New repo text in plain English; do not rewrite existing German code. Keep the T7 out of it (not connected). Gate/test output goes to files under runs/ in full, never piped through head/tail.

CLEAN CODE (the user's top priority - he wants to read and explain every line):
- Small functions with one job and clear English names; type hints; a short docstring per module and per public function that says WHAT it does and WHY it exists (safety reason, robot fact), not how.
- No clever tricks, no hidden magic numbers: every limit/threshold is a named constant with a one-line comment giving its source (plan section, Unitree example, measurement).
- Comment only lines whose purpose is not obvious; match the repo's existing style; keep files short (split rather than grow past ~300 lines).
- Fail loudly with a clear message that tells the operator what to do, never silently.

MEANINGFUL TESTS (few, but each one proves something that matters; not too many):
- Every test has a descriptive name (test_<what>_<expected>) and a one-line docstring: "Proves: ... / Catches: <the concrete bug it would catch>".
- Arrange - Act - Assert layout, one behaviour per test, fakes instead of a real robot.
- Layers: unit (one function, pure logic, seconds), integration (several parts together on the loopback sim), e2e (the full chain 84/86 -> 85 -> sim with the 40k or replay server).
- The safety-critical checks (limits, frozen ramp-in, release speed, stale stop, no-publisher, motion gate) must be shown to FAIL on a deliberately broken version first (mutation check) - note it in the log.

PROGRESS THE USER CAN FOLLOW: when you start and when you finish, append one plain-language line to \`.planning/STATUS.md\` stamped with the real clock time (run \`date "+%Y-%m-%d %H:%M"\`, never estimate), e.g. "16:40 Track B1 (safety layer) started: writing the joint-limit and speed checks." / "17:55 Track B1 done: real_safety.py clamps every command to the joint limits and to 0.3 rad/s; 9 unit tests pass (2 mutation checks seen failing)." No jargon without a 3-word explanation.`

const VERDICT = {
  type: 'object',
  properties: {
    verdict: { type: 'string', enum: ['GO', 'NO-GO', 'OPEN'] },
    findings: { type: 'array', items: { type: 'object', properties: {
      severity: { type: 'string', enum: ['blocker', 'major', 'minor'] },
      summary: { type: 'string' }, file: { type: 'string' }, fix: { type: 'string' } },
      required: ['severity', 'summary', 'fix'] } },
    review_file: { type: 'string' },
  },
  required: ['verdict', 'findings'],
}
const BUILD = {
  type: 'object',
  properties: {
    files_changed: { type: 'array', items: { type: 'string' } },
    verify_commands: { type: 'array', items: { type: 'string' } },
    verify_passed: { type: 'boolean' },
    notes: { type: 'string' },
  },
  required: ['files_changed', 'verify_passed', 'notes'],
}
const blocking = v => v && v.findings.filter(f => f.severity !== 'minor')

// Stages let the main session run plan, each track and the finish as separate workflows (one notification per milestone).
// args: { stage: 'plan' | 'build' | 'finish' | 'all', tracks: [...], planRoundStart: N, trackResults: [...] }; no args = old behaviour.
const STAGE = (args && args.stage) || 'all'
const PLAN_ROUND_START = (args && args.planRoundStart) || 1
const PLAN_REVISE_FIRST = `${COMMON}

REVISE revision 6 of \`.planning/real-robot/PLAN.md\` in place (only that file; no code, no sims). Do NOT rewrite it from scratch: rev 6 is current (Monday 10-19 scope, five build tracks, motion gate) and everything the round-1 reviewer did not flag stays as it is. Plan review rev 6 - round 1 (in \`.planning/reviews/real-robot-plan.md\`) is NO-GO with no blockers and three majors - fix each in the plan text:
- RV6-1: table clearance of the scripted arm paths at M4-M6 (release / ready / hands-open paths must not sweep through the table; exit via the ready pose; a precondition before M4-M6).
- RV6-2: the factory-pose record must be taken at the start of Part C, in the robot mode Part C actually uses.
- RV6-3: honest push-1 timing (the clock now is ~15:40 Sunday; the reviewer estimated ~21:00-22:00, not 20:00) and a corrected fallback; push 1 happens when tracks A and E are GO.
Fold the 13 minors into the track tasks where cheap (the reviewer said they need no extra review round). Keep the "Build tracks" section consistent and disjoint. Bump the header to "revision 6.1" and add a "Review response (rev 6 round 1)" entry naming how each major was fixed. Return what changed.`

let planReview = null
let planNote = ''
if (STAGE === 'all' || STAGE === 'plan') {
// ---------------- Plan ----------------
phase('Plan')
const PLAN_TASK = `${COMMON}

Write revision 6 of \`.planning/real-robot/PLAN.md\` (only that file; no code, no sims). The situation changed AGAIN (user, Sun 2026-10-18 ~15:00):
- The robot day is **Monday 2026-10-19**, not Sunday. Cristina (acristinaa) and Saad (saadsheikh-tech) operate; Finnegan is in Bielefeld (reachability unknown, assume none).
- Build window: **Sunday 2026-10-18 ~15:30 until Monday 09:00, running unattended overnight** on Finnegan's laptop. Two pushes (Finnegan pushes): **push 1 (~20:00)** = Tier A + handover kit (+ the GitHub Release), so Cristina and Saad can run setup + self-test on Sunday evening; **push 2 (~07:30 Monday)** = the motion code (B1, B2, C) ONLY if every review and the final gate are GO - otherwise it is not pushed and Monday stays read-only. Operators \`git pull\` before 09:00. Order the timeline and cut list around that: tracks A and E must be GO by ~19:30; the final gate must finish by ~06:30.
- **The user wants the VLA to actually attempt the grasp on the real robot on Monday.** So Tier B comes back: the safety layer \`g1sim/real_safety.py\`, the adapter \`scripts/86_real_run.py\` (modes at least: release, ready, replay, shadow, run, hands-open), the sim stand-in \`scripts/85_arm_sdk_sim.py\` that makes the existing DDS sim obey \`rt/arm_sdk\`, and a sim-to-sim test of exactly that code path including fault injection. Restore every safety fix from the earlier review rounds (N1 frozen q0 for the whole ramp-in, N2 release moves to the recorded factory pose at <=0.3 rad/s before ramping out, N4 protected factory-pose record, B1 demo-envelope clip only in policy/replay phases, M2 hold measured angles on contact/lead faults, waist 13/14 always held, hands opened actively on soft exit, stale camera/policy/state stop, 200/400 Hz thresholds, foreign-writer check by samples) - they are listed in \`.planning/reviews/real-robot-plan.md\` and Appendix D.
- Keep Tier A (84, real_io, real_camera, no-publisher rule for 84/89/71) and the handover kit (runbook, setup script for macOS + Ubuntu with a Python 3.10 robot env, self-test 89, model as GitHub Release \`robot-day-2026-10-18\` already packed: 40k tar + sha256).
- **Motion gate for non-author operators:** 86 refuses unless (a) \`docs/ROBOT_DAY_MOTION.json\` = allowed + pinned commit, set ONLY by Finnegan after all code reviews are GO, (b) every runtime-loaded repo file and the JSON match that commit, (c) both operators confirm, (d) the robot hangs in the gantry/harness for every motion step (operator confirms), (e) speed factor <= 0.5, (f) no kill test by non-authors. Bring-up order Monday: 84 read-only -> release -> ready (arms in air) -> slow replay of one demo -> VLA shadow -> VLA live slow; each step with an abort rule and a runbook box for "release refused/crashed".
- Walking: \`scripts/70_real_goto.py\` exists and stays untouched; the robot is placed at the table by the owner's remote. 70 is NOT part of Monday's runbook (mention as optional owner-supervised demo only if the owner wants it).
- Policy server runs on a teammate laptop (macOS MPS or Linux CUDA; add the cuda device line to 81).
- Add a section **"Build tracks"** with exactly five tracks A, B1, B2, C, E: for each the exact files it OWNS (disjoint across tracks), the interface it codes against (contracts in section 3, extended for real_safety and 86), its verify command(s) and done criterion. Suggested split: A = real_io, real_camera, 84, their tests, fake_camera, no-publisher tests; B1 = real_safety + its unit tests (pure logic, fakes only); B2 = 86 + its unit tests (against real_safety's contract with fakes); C = 85 + tests/helpers/sim_host.py + the sim-to-sim/fault integration test file; E = runbook, setup script, 89, loopback helper, test data, reference frame, the 81 cuda line.
- Re-time to Sunday 15:30 - Monday 06:30 with clock-time checkpoints (push 1 ~20:00, final gate ~06:30, push 2 ~07:30); honest expectation (sim-only VLA likely misses at first; success = safe motion, clean replay, first attempts, real data).
Return a short summary.`

for (let round = PLAN_ROUND_START; round < PLAN_ROUND_START + 3; round++) {
  // args.reviewFirst: the plan was already revised before this run, so the first round goes straight to the review.
  const skipPlanner = args && args.reviewFirst && round === PLAN_ROUND_START
  const planOut = skipPlanner ? '(plan revised before this run)' : await agent(round === 1 ? PLAN_TASK : (round === PLAN_ROUND_START && round === 2) ? PLAN_REVISE_FIRST
    : `${COMMON}\n\nRevise \`.planning/real-robot/PLAN.md\` again (only that file). The plan review round ${round - 1} is NO-GO; its findings are in the latest "Plan review rev 6" section of \`.planning/reviews/real-robot-plan.md\`. Fix every blocker and major (or refute with re-derived evidence in a "Review response" section). Keep the "Build tracks" section consistent. Return what changed.`,
    { label: `planner:rev6-r${round}`, phase: 'Plan', agentType: 'planner', model: 'opus' })
  planNote = planOut
  planReview = await agent(`${COMMON}

You are an independent plan reviewer; you did not write the plan. Review the current revision (6.x) of \`.planning/real-robot/PLAN.md\` (round ${round}) for a Monday 2026-10-19 session where two non-author operators run new motion code on a real Unitree G1 EDU 29 DoF (BrainCo hands) without the author present. Judge: (1) is the safety design sufficient for that (gate, gantry, ramps, release, faults, stale checks, hands, waist 13/14, what happens when our program dies) and are the earlier fixes (in \`.planning/reviews/real-robot-plan.md\`) all preserved; (2) are the build tracks (A, B1, B2, C, E1 in parallel; E2 after E1 is GO) truly disjoint in files, with contracts precise enough to build in parallel, and verify commands that can run on this Mac tonight; (3) does the timeline fit (build from ~17:00 Sunday 2026-10-18 to Monday 09:00 unattended overnight; push 1 when tracks A and E1 are GO, expected ~22:30, latest 23:30; final gate ~06:30; push 2 ~07:30-08:00; operators pull before 09:00 - the plan's section 6 is the reference). Re-derive load-bearing facts from code/sources, do not trust the plan. Do not edit the plan or code; run no sims. Append "Plan review rev 6 - round ${round}" with each finding (severity, evidence, fix) and a final GO/NO-GO/OPEN to \`.planning/reviews/real-robot-plan.md\`.`,
    { label: `plan-review:r${round}`, phase: 'Plan', agentType: 'reviewer', model: 'opus', schema: VERDICT })
  if (!planReview) { log(`plan review round ${round} died - gate OPEN`); break }
  log(`plan review round ${round}: ${planReview.verdict} (${blocking(planReview).length} blocker/major)`)
  if (planReview.verdict === 'GO') break
}
if (!planReview || planReview.verdict !== 'GO') {
  return { stage: 'plan', planReview, planNote, message: 'Plan gate not GO after 3 rounds - build not started.' }
}
if (STAGE === 'plan') return { stage: 'plan', planReview, planNote }
}

// E2 is not listed: it starts inside the pipeline once E1 is GO (plan section 5).
const TRACKS = (args && args.tracks) || ['A', 'B1', 'B2', 'C', 'E1']
const buildPrompt = t => `${COMMON}

You are the implementer for **track ${t}** of the real-robot build. Read \`.planning/real-robot/PLAN.md\` fully (it passed review), especially section 3 (contracts), the rules, and the "Build tracks" section. Build EXACTLY the files that track ${t} owns - do not create or edit files owned by other tracks (they are being built in parallel right now; code against their contracts and use fakes in your tests). Follow the surrounding code style. Tests first where the plan says "seen failing first": record the failing run in the log. Run your track's verify command(s); output to runs/ files. Five tracks build at once on one Mac: only track C may start a MuJoCo/DDS sim or GPU process during this phase (several at once froze processes in the graphics driver before); all other tracks test with fakes and leave sim runs to integration. If a verify step needs another track's file that does not exist yet, write the test, mark that step "pending integration" in your notes, and make everything else pass. Return the files you changed, the verify commands and whether they passed.`
const reviewPrompt = (t, round) => `${COMMON}

You are an independent code reviewer for **track ${t}** (round ${round}); you did not write it. Read the plan (\`.planning/real-robot/PLAN.md\`, section 3, the rules, "Build tracks") and the files track ${t} owns. Give two verdicts: did the code do what the plan says, and is anything in the plan wrong for real hardware. Focus on safety correctness (limits, ramps, frozen q0, release, faults, stale checks, no-publisher rule, motion gate), tests that can actually fail (try a mutation on the most important check), anything that would break on the operators' laptops, and CLEAN CODE: could a second-year student read and explain every function and every test? Unreadable code, unexplained constants or tests without a 'Proves/Catches' docstring are majors. Re-run the track's unit tests yourself (output to runs/). Do not edit code. Append "Track ${t} - round ${round}" to \`.planning/reviews/real-robot-code-${t}.md\` (this track's own findings file, plan m6-12) with each finding and a GO/NO-GO/OPEN line.`
const fixPrompt = (t, v) => `${COMMON}

You are the implementer for **track ${t}**. The code review is ${v.verdict}. Fix every blocker and major listed below (or refute one with re-derived evidence written into \`.planning/reviews/real-robot-code-${t}.md\` under "Track ${t} - response"); take minors that cost < 15 min. Only touch track ${t}'s files. Re-run the track's verify commands.
Findings:
${JSON.stringify(v.findings, null, 1)}`

let trackResults = (args && args.trackResults) || []
if (STAGE === 'all' || STAGE === 'build') {
phase('Build')
log(`Building ${TRACKS.join(', ')} (A read-only checklist, B1 safety layer, B2 adapter, C sim test stand, E handover kit).`)
// One track: build -> review -> up to 2 fix/review rounds.
const buildTrack = async t => {
  log(`track ${t}: building`)
  const built = await agent(buildPrompt(t), { label: `build:${t}`, phase: 'Build', agentType: 'implementer', model: 'opus', schema: BUILD })
  log(`track ${t}: built (${built ? built.files_changed.length : 0} files, verify ${built && built.verify_passed ? 'passed' : 'NOT passed'}) - now in review`)
  let v = await agent(reviewPrompt(t, 1), { label: `review:${t}:r1`, phase: 'Build', agentType: 'reviewer', model: 'opus', schema: VERDICT })
  for (let round = 2; round <= 3 && v && v.verdict !== 'GO'; round++) {
    await agent(fixPrompt(t, v), { label: `fix:${t}:r${round - 1}`, phase: 'Build', agentType: 'implementer', model: 'opus', schema: BUILD })
    v = await agent(reviewPrompt(t, round), { label: `review:${t}:r${round}`, phase: 'Build', agentType: 'reviewer', model: 'opus', schema: VERDICT })
  }
  log(`track ${t}: ${v ? v.verdict : 'review died (OPEN)'}`)
  return { track: t, built, verdict: v ? v.verdict : 'OPEN (review agent died)', open: v ? blocking(v) : null }
}
trackResults = (await pipeline(TRACKS,
  t => buildTrack(t),
  async (r, t) => (t === 'E1' && r.verdict === 'GO') ? [r, await buildTrack('E2')] : [r]
)).flat().filter(Boolean)
if (STAGE === 'build') return { stage: 'build', tracks: trackResults }
}

// ---------------- Integrate ----------------
phase('Integrate')
log('All tracks reviewed. Integration: whole test suite + the full chain in the sim with the 40k model + fault cases + fresh-laptop setup.')
const integ = await agent(`${COMMON}

Integration of the real-robot build (all five tracks A, B1, B2, C, E are written; per-track results: ${JSON.stringify(trackResults.map(r => r && { track: r.track, verdict: r.verdict }))}).
1. Run the whole existing suite: \`scripts/run_tests.sh\` > runs/real_int_suite.log 2>&1 and the new real-robot tests; everything that passed before must still pass.
2. Run every verify command marked "pending integration" and the plan's integration checks: 84 against the loopback sim host (0 rt/arm_sdk samples, own writers 0); 86 release/ready/replay/shadow/run through 85 on loopback with the 40k checkpoint server (\`~/dev/g1-ext/smolvla-runs/run1/checkpoints/040000/pretrained_model\`, MPS) and with a replay server; all fault injections from the plan; the ramp-in "< 0.03 rad" check; release peak speed <= 0.35 rad/s; motion gate refusals (no JSON, wrong commit, edited file, speed > 0.5).
3. Run the handover kit on a fresh prefix ~/dev/g1-ext/robot-day-test: setup robot-env, policy-env (may take long, run in background and wait), --release-file with the packed tar at ~/dev/g1-ext/release/smolvla_run1_040000.tar (+ .sha256), then 89 until SELFTEST: PASS.
Fix integration bugs where the cause is clear and small (any file), recording each fix in \`.planning/reviews/real-robot-integration.md\` under "Integration fixes" so a reviewer can re-check it. Return what ran, what passed, what was fixed, what is still failing.`,
  { label: 'integrate', phase: 'Integrate', agentType: 'implementer', model: 'opus', schema: BUILD })

// ---------------- Dry walk ----------------
phase('Dry walk')
log('Dry walk: a fresh agent plays Cristina and follows the runbook word for word against the sim.')
const walk = await agent(`${COMMON}

You are Cristina, an operator who did NOT write this code. Follow \`docs/ROBOT_DAY_RUNBOOK.md\` literally, as written: Part C (the motion steps) in full, plus only those Part A/B lines that changed since their read-only walk before push 1 (\`.planning/reviews/real-robot-runbook-walk-AB.md\`, plan 6.1 seam 4), but against the loopback sim (sim host / 85 on lo0) instead of the robot, using the fresh prefix ~/dev/g1-ext/robot-day-test if it exists. Do not read the source code to work around unclear steps - if a command fails or an instruction is unclear, that IS the finding. Do not edit any file. Write every unclear, wrong or missing instruction (step, what happened, what you expected) to \`.planning/reviews/real-robot-runbook-walk.md\` and return a verdict.`,
  { label: 'dry-walk', phase: 'Dry walk', agentType: 'reviewer', model: 'opus', schema: VERDICT })
let walkFix = null
if (walk && walk.verdict !== 'GO') {
  walkFix = await agent(`${COMMON}\n\nFix the runbook dry-walk findings in \`.planning/reviews/real-robot-runbook-walk.md\` (runbook, setup script, 89, CLI help texts - whichever is wrong). Record each fix under "Dry-walk fixes" in that file. Re-run the steps you changed.\nFindings:\n${JSON.stringify(walk.findings, null, 1)}`,
    { label: 'fix:dry-walk', phase: 'Dry walk', agentType: 'implementer', model: 'opus', schema: BUILD })
}

// ---------------- Final gate ----------------
phase('Final gate')
log('Final gate: a judge re-checks everything and decides per file GO / NO-GO and whether motion can be unlocked.')
const judge = await agent(`${COMMON}

Final judge over the real-robot build for Monday 2026-10-19 (non-author operators, real G1). Read the plan, every \`.planning/reviews/real-robot-code-<track>.md\`, \`.planning/reviews/real-robot-integration.md\`, \`.planning/reviews/real-robot-runbook-walk-AB.md\`, \`.planning/reviews/real-robot-runbook-walk.md\`, the integration result below, and re-check the contested or late fixes yourself (integration fixes and dry-walk fixes were NOT reviewed by a track reviewer - review them now). Re-run the new real-robot unit tests and the sim-to-sim test (output to runs/). Also screen every NEW file for dead code: top-level functions/classes never called from code (tests only count if the function is a deliberate test helper), unused top-level imports, unused flags - list any as minor findings. Decide per file GO / NO-GO, and overall: can Finnegan safely unlock the motion gate, or should Monday stay read-only? Write "Final gate" to \`.planning/reviews/real-robot-final.md\` and update \`.planning/STATUS.md\` with a clear summary for Finnegan (what is done, what failed, the exact commit list to push, the release command, and his unlock decision).
Integration result: ${JSON.stringify(integ)}
Dry-walk fix: ${JSON.stringify(walkFix)}`,
  { label: 'judge', phase: 'Final gate', agentType: 'judge', model: 'opus', schema: VERDICT })

phase('Testing guide')
log('Writing docs/TESTING_GUIDE.md: a short explanation of every new test for Finnegan.')
const guide = await agent(`${COMMON}

Write \`docs/TESTING_GUIDE.md\` for Finnegan, a student taking an automated-testing course who wants to read, understand and pitch the tests. Keep it SHORT (2-3 pages):
1. Half a page: unit vs integration vs e2e vs eval, Arrange-Act-Assert, what a mutation check is - using THIS project as the example.
2. A table of every test added tonight (file, test name, layer, "Proves", "Catches", how to run it), grouped by part A/B1/B2/C/E. Take Proves/Catches from the docstrings. The final gate has already passed, so do NOT edit any test or code file; if a docstring is unclear, explain it better in the guide and list it under "Docstrings to improve".
3. A guided read of the 3 most important safety tests (frozen ramp-in, motion gate refusal, no-publisher rule): the test code with a plain explanation per block.
4. The one command that runs everything, and how to read its output.
Do not change any test logic. Append a STATUS line when done.`,
  { label: 'testing-guide', phase: 'Testing guide', agentType: 'implementer', model: 'opus' })

return {
  testingGuide: !!guide,
  plan: planReview && planReview.verdict,
  tracks: trackResults.map(r => r && { track: r.track, verdict: r.verdict, open: r.open }),
  integration: integ && { passed: integ.verify_passed, notes: integ.notes },
  dryWalk: walk && walk.verdict,
  judge,
}
