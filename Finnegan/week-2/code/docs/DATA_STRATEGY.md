# DATA STRATEGY — How the robot actually learns

> Companion to `PLAN.md`. PLAN.md answers *what we build*, PROGRESS.md *where we are*.
> This file answers the question that neither of them answers cleanly:
> **where does the training data come from?**
>
> Created 2026-09-09 after the question "can we train in MuJoCo and use that data on the
> real G1, and fine-tune a VLA on openpi?"

---

## 0. The one-paragraph answer

Yes — but not the way it sounds. You do **not** train a VLA inside MuJoCo the way you
trained the standing policy with RL. You use MuJoCo as a **demonstration factory**: the
Track A pipeline we already have (VLM → 3D point → IK → grasp) is run thousands of times
with randomized objects, poses and lighting, and every successful run is written out as a
training episode. The VLA is then trained **offline** on that pile of episodes. Simulation
supplies volume; a much smaller set of real Vision-Pro teleop episodes supplies realism.
The two are **co-trained together**, not sequentially. This is a documented recipe, not
speculation, and it is the single biggest change to our plan.

---

## 1. What PLAN.md and PROGRESS.md already got right

Both documents are correct on the hard part and should not be rewritten:

- **`PROGRESS.md` § "WHY RL DOES NOT WORK FOR 'TAKE THE BOTTLE'"** is right.
  Sparse reward + 26 DOF = no learning signal. Nobody trains manipulation with pure RL
  from scratch. That section stays exactly as it is.
- **`CLAUDE.md` rule 1** — "never train a VLA from zero, always fine-tune" — right.
- **`PLAN.md` § "Do not train perception in MuJoCo"** — right. MuJoCo's renderer will not
  teach a VLM what a bottle is. That is not what we are proposing here.
- **`PLAN.md`** picking π₀ / `openpi` (Apache-2.0) over UnifoLM-VLA-0 (CC BY-NC-SA) —
  right, and load-bearing for the SAP business case.

## 2. Where they are now out of date

Both documents treat simulation as: *model building + IK development + eval harness +
sim2sim safety gate* — and explicitly **not** as a source of training data. PLAN.md
Phase 6 lists exactly one data path: `xr_teleoperate` on the real robot.

**That is the gap.** Simulation is also a legitimate — and for us, the *primary* — source
of VLA training data. The evidence:

- **openpi's own flagship example is trained on simulated data.** `examples/libero/` is
  the repo's reference fine-tuning path, LIBERO is 100 % simulated, and
  **π₀.₅-LIBERO is a released, state-of-the-art checkpoint.** Physical Intelligence
  themselves ship a sim-trained model.
- **Sim-and-Real Co-Training** (co-training.github.io) tests this on two embodiments
  including a humanoid, ~14 tasks. Findings that matter to us:
  - use **significantly more sim data than real data**; tune the sampling ratio α
  - a **perfect digital twin is not required** — "digital cousins" are enough
  - **exact camera alignment is not critical** (helpful, not required)
  - **object and placement diversity in sim is what pays off**
- π₀ fine-tuned from sim-pretrained weights reaches ~60 % on tasks where the
  no-sim-pretraining baseline scores **0 %**, in the low-real-data regime — which is
  precisely our regime.

**Consequence:** Track B stops being a stretch goal gated on a VR headset. We can start
generating VLA training data **this week, on the M4, with no robot and no headset.**

---

## 3. The mechanism, concretely

### 3.1 Track A is the demonstration generator

We already have the expensive parts:

| Piece | Status | File |
|---|---|---|
| Table scene, bottle + distractors | DONE | `models/g1_brainco/scene_table_hands.xml` |
| Head + 2 wrist cameras | DONE | (Phase 1a) |
| Pixel → 3D grasp point, 0.9 cm error | DONE | `scripts/13_grasp_target.py` |
| Revo2 hands, 12 actuated DOF | DONE | `scripts/15_attach_hands.py` |
| IK grasp point → joint angles | **Phase 2b, open** | — |

Once 2b closes, one script loop produces data forever:

```
for episode in range(N):
    randomize(object type, position, rotation, distractors, lighting, table texture)
    target = grasp_target(...)          # geometry, already verified
    traj   = ik_solve(target)           # Phase 2b
    record every control step:
        head_cam RGB, wrist_cam RGB, qpos (state), ctrl (action), instruction string
    if grasp_succeeded(): write_episode()
    else:                 discard
```

**Only successful episodes are kept.** The policy imitates what worked. Failures are
thrown away, which is why sparse reward — fatal for RL — is harmless here.

### 3.2 The output format is trivial

> **Update 2026-09-14 (decided):** the stack is **openpi** (π₀.₅ base + LoRA), so the schema below — openpi's LIBERO
> example, LeRobot dataset **v2.1** — is the one we write, with three cameras (`head_image`, `left_wrist_image`,
> `right_wrist_image`), 26-D `state` and `actions` (layout `unitree_lerobot` `Unitree_G1_Brainco`, module
> `g1sim/action_space.py`). openpi computes its own norm stats (`scripts/compute_norm_stats.py`, quantiles for π₀.₅).
> Never mix v2.1 and v3.0 datasets.

openpi consumes **LeRobot datasets**. From `examples/libero/convert_libero_data_to_lerobot.py`,
the entire required schema is:

```python
features={
    "image":       {"dtype": "image",   "shape": (256, 256, 3)},   # head cam
    "wrist_image": {"dtype": "image",   "shape": (256, 256, 3)},   # wrist cam
    "state":       {"dtype": "float32", "shape": (N_STATE,)},      # proprioception
    "actions":     {"dtype": "float32", "shape": (N_ACTION,)},     # 26 for g1_brainco
}
# plus per frame:  "task": "pick up the red bottle"
```

Comment in that file: *"OpenPi assumes that proprio is stored in `state` and actions in
`action`."* That is the whole contract. We write MuJoCo rollouts straight into it.

### 3.3 Throughput — using our own measured numbers

From `PROGRESS.md`: physics **15 079 steps/s**, offscreen render **56 fps at 640×480**,
12 cores. openpi trains at 256×256, which is ~6× cheaper per frame than what we measured.

Rough budget for a 10 s episode at 30 Hz with 2 cameras (600 renders):
**~1 000 episodes in one to two hours** across 8 worker processes. Overnight gets us
into five figures.

**Storage:** raw frames would be ~120 GB per 1 000 episodes. LeRobot stores video-encoded,
so budget ~5–15 GB. Do not write PNGs.

### 3.4 What sim data can and cannot teach

| Learns well from sim | Does **not** learn from sim |
|---|---|
| Reach/approach trajectories | Photorealistic appearance |
| Grasp timing and pre-shape | True contact/friction/slip dynamics |
| Language → which object (grounding) | Deformable and articulated objects |
| Recovery from bad approach angles | Real actuator latency and backlash |
| Multi-step ordering | Real hand tendon compliance |

The right-hand column is exactly what the small real dataset is for.

---

## 4. Apple Vision Pro — what it is and is not

### 4.1 The misconception to clear up first

**The Vision Pro's LiDAR and cameras are not the training data.**

In `xr_teleoperate` the headset is an **input device** — a high-quality hand-tracking
joystick. It streams your hand and wrist poses to the Linux machine, which retargets them
onto the G1's arms and Revo2 fingers. The recorded episode contains the **robot's** head
and wrist camera images and the **robot's** joint states. The AVP contributes the
*actions*, never the *observations*.

Practical consequence: we are not buying a Vision Pro for its sensors. We would be buying
it as a controller. A Quest 3 (~€550) does the same job in the same repo for roughly a
tenth of the price. **If the headset is bought for teleop only, the Quest 3 is the
rational purchase** — the Vision Pro's hand tracking is nicer, not categorically different.

### 4.2 Downsides of the real-teleop path

1. **Ubuntu 20.04/22.04 + Python 3.10 only.** This is `PROGRESS.md` Blocker 3, and it now
   arrives *earlier* than planned. It also collides head-on with the SAP one-way-network
   constraint: if the only GPU is SAP hardware on site, the fine-tuned model is born
   inside a network it can never leave.
2. **No force feedback.** The operator cannot feel contact. Grasps are inconsistent —
   too hard, too soft, slightly off — and the policy faithfully learns that inconsistency.
   This is the single largest quality problem with VR teleop data.
3. **Latency.** WebRTC video to the headset plus network jitter puts 20–50 ms+ in the
   loop. Operators overcorrect, and the correction oscillation is baked into every
   episode as if it were intentional.
4. **Retargeting is lossy.** Human hand → Revo2 (6 actuators/hand, rest coupled by
   `equality` constraints, per `PROGRESS.md`) is not a bijection. Your fingers and the
   robot's do not do the same thing.
5. **The data is welded to the hardware configuration.** Change the hand mount, move a
   camera, switch 29 → 23 DOF, and part of the dataset is invalidated.
   → **This makes step 1c2 (mount fine-tuning) a hard prerequisite. Do not record a
   single episode, sim or real, before the mount is frozen.**
6. **Reset cost dominates.** Someone physically puts the ball back, every single time.
   In practice resets are ≥50 % of wall-clock. Sim resets are free and instant.
7. **Wear and risk.** Several hundred demos is hours of arm cycling on a €40 k robot
   with a human inside its workspace.
8. **Walking cannot usefully be teleoperated this way.** See §5.
9. **Operator variance.** Two people solve the same task differently; the policy averages
   two strategies into one that works for neither. Prefer one operator per task.

### 4.3 What real teleop is genuinely irreplaceable for

Real contact physics, real lighting and sensor noise, real hand compliance, and the real
failure modes we cannot imagine well enough to script. That is worth **tens** of episodes,
not thousands — which is exactly what the co-training result predicts.

---

## 5. The target chain, decomposed

> "go into the room, search for a ball, pick it up, carry it somewhere, put it down,
> then trigger an action in the ERP system"

This sounds like one enormous learned behaviour. It is not, and must not be built as one.
It is six steps, of which **only two need training data at all**:

| # | Step | How it is actually built | Demos needed |
|---|---|---|---|
| 1 | Walk into the room | **Factory G1 locomotion controller.** Already works, cost €0. | **0** |
| 2 | Search for the ball | VLM on the head camera + a scripted look-around. Internet-pretrained. | **0** |
| 3 | **Pick up the ball** | Track A now; VLA later | **~200–500** |
| 4 | Carry it to the target | Factory locomotion + nav goal | **0** |
| 5 | **Put it down** | Track A now; VLA later | **~100–300** |
| 6 | Trigger the ERP action | **An HTTP call.** ~20 lines of Python. | **0** |

The glue is an **orchestrator**: a VLM/LLM that reads the instruction, decides which skill
runs next, and calls step 6 when the place-skill reports success. That architecture is
**Hi Robot** ([arXiv 2502.19417](https://arxiv.org/abs/2502.19417)) — System 2 plans,
System 1 (π₀) acts. `PLAN.md` already names it. Cite it, do not reinvent it.

**Two consequences worth saying out loud:**

- **Do not teleoperate walking.** The G1 walks from the factory. Recording locomotion
  demos would burn the entire data budget reproducing something already in the box.
  `PLAN.md` § "Cut locomotion entirely" was right and stays right.
- **Step 6 is the SAP business case, and it is the easiest step in the list.** The
  interesting SAP question is not "can a robot call an API" — it is *what the robot
  reports, what the ERP does with it, and what happens when the grasp fails.* That
  deserves design effort. The API call does not.

### 5.1 So how much data, really?

For **one** manipulation skill on our own embodiment:

| Source | Guidance |
|---|---|
| π₀ | 100–1 000 demos per task |
| π₀.₅ | 200–500 for single-task adaptation on a known embodiment |
| LeRobot practice | ~50 episodes is the floor for *something* to work |
| Co-training paper | 20–400 real demos, plus much more sim |

**Our plan: ~2 000–5 000 sim episodes + ~50–150 real episodes per skill, co-trained.**

Real-teleop time for 100 episodes, honestly: ~45 s per demo plus reset ≈ 15–20 demos/hour
with two people. **100 demos ≈ 6 hours.** 500 demos ≈ 30+ hours. That number is why
sim-first is not a shortcut — it is the only version of this that fits a student project.

---

## 6. The blocker this creates — read this before generating any data

`PROGRESS.md` records step **1d (action-space mapping) as OPEN**, with a specific
unit mismatch:

> Sim controls the hand in **radians** (ctrlrange 0…1.46), the real `g1_brainco` stack
> per issue #46 expects **normalized 0.0…1.0**.

Under the old plan 1d was low-priority — it blocked only the real robot. **Under this plan
it blocks everything.** Every sim episode we generate stores an `actions` vector. If that
vector is in the wrong units or the wrong joint order, then 5 000 episodes are 5 000
pieces of silently-wrong training data, and the failure will only surface on hardware.

**Order of operations is therefore fixed:**

```
1c2  freeze the hand mount        ← in progress, needs your eye
1d   define the 26-DOF action vector: order + units + normalization
2b   closed-loop IK
---- only now ----
generate sim episodes
```

---

## 7. Recommended path

**Now, on the M4, no new hardware:**
1. Finish **1c2** (mount) and **2a2** (VLM in the loop). Both already have their tooling.
2. Do **1d** properly and write it down as a single source of truth — one module that
   converts between MuJoCo `ctrl` and the real `g1_brainco` vector, used by everything.
3. Build **2b** closed-loop IK, then **2c** the eval harness.
4. Wrap Track A in a randomization loop → LeRobot writer. **This is the data factory.**
   Validate it by training a small ACT or Diffusion Policy — cheap, fast, and it proves
   the dataset is well-formed before we rent a GPU.

**Then, one rented RTX 4090 (~€1–2/h, RunPod/vast.ai — do not buy a GPU):**
5. LoRA fine-tune π₀.₅ on sim episodes only. Evaluate in the 2c harness against Track A.
   **Track A is the baseline the VLA has to beat**, and if it does not, that is a
   perfectly good result to report.

**Then, when the robot is here and the mount is frozen:**
6. Record 50–150 real episodes of the same skill (headset or controller mode).
7. Co-train sim + real, tuning the ratio α. Compare all three in the same harness.

**Buy decision:** a **Quest 3** and a **Linux mini-PC with an NVIDIA GPU** beat a Vision
Pro for this project. The Linux box also resolves Blocker 3 permanently and keeps the
fine-tuned model outside the SAP network, which the SAP one-way constraint otherwise
makes impossible.

---

## 8. Honest expectations

- Sim-only VLA → real robot, zero-shot: **expect it to fail.** Co-training is not optional
  polish, it is the mechanism.
- Contact-rich manipulation sim2real success rates of 5–10 % are normal
  (`CLAUDE.md` already says this). A 30 % pick success on hardware would be a good result.
- **Track A stays the deliverable.** It needs no training data, no GPU and no headset, and
  it is debuggable in a way an end-to-end policy is not. The VLA work is what makes the
  project interesting; Track A is what makes it *finish*.
- The largest cost in this project remains **data collection and its plumbing**, not
  compute. Nothing here changes that — it just moves most of the cost into simulation,
  where it is cheap.

## Sources

- openpi — https://github.com/Physical-Intelligence/openpi (Apache-2.0; LoRA >22.5 GB VRAM,
  inference >8 GB, full fine-tune >70 GB; Ubuntu 22.04 only; LeRobot dataset format)
- openpi LIBERO example — sim-trained π₀.₅-LIBERO checkpoint
- Sim-and-Real Co-Training — https://co-training.github.io/
- Hi Robot — https://arxiv.org/abs/2502.19417
- xr_teleoperate — https://github.com/unitreerobotics/xr_teleoperate (AVP / Quest 3 /
  PICO 4 Ultra; Ubuntu 20.04/22.04 + Python 3.10; G1 23 & 29 DOF; BrainCo hand)
- unitree_lerobot — https://github.com/unitreerobotics/unitree_lerobot
