# Finnegan's work – status 7 Oct 2026

Simulation work on our Unitree G1 (MuJoCo, BrainCo Revo2 hands). The code lives in my repo `finneganadam-maker/UnitreeG1-Robot`;
ask me for access. This folder is a snapshot so everyone can see where things stand.

**Short version:** in simulation the robot can now pick up a volleyball with both hands and drop it into a basket, 50 out of 50
times. That is a **programmed "teacher"**, not AI yet. Its job is to record training examples. Tomorrow we record them and train
the first VLA (vision-language-action model) on them.

---

## What was built today

| What | Result |
|---|---|
| **Volleyball scene** | Ball (21 cm, 270 g) on a small tube stand so it can't roll, a basket on the other side of the table |
| **Teacher: two-hand pick and place** | Both palms cradle the ball from below, all five fingers curl gently, lift, **turn at the waist**, release into the basket. **10/10** at the normal position, **50/50** with the ball moved randomly by up to ±2 cm |
| **Lab View** | A browser page that shows simulation runs live (trial count, current step, success rate, failure reasons) plus the history of all runs |
| **SmolVLA test on a MacBook (M4 Pro, 24 GB)** | Training works overnight: about 5 h for 20,000 training steps. One decision takes ~200 ms and covers 50 movement steps |
| **Robot specs confirmed** | G1 EDU 29 DoF, D435i + D455 on a 2-axis head, Livox Mid-360, Jetson Orin NX 16 GB, Thunderobot backpack (i7 + RTX) |

Pictures and video are in [`media/`](media/):
- `volleyball_demo_video.mp4`: one full run (25 s)
- `volleyball_teacher_squeeze.png`: the robot holding the ball (front and side)
- `volleyball_scene_overview.png`, `volleyball_scene_head.png`: the scene, and what the head camera sees

---

## How the whole system fits together

```
"Hey Unitree, take the ball from table A and put it in the box"
   → Planner (AI language model, runs locally or on SAP's own AI service, not OpenAI)
   → Orchestrator (runs the steps in order, checks each one, reports to SAP)
       → BotBrain: map + navigation (walk to table A / to the box)   – legs only
       → VLA (π₀.₅ or SmolVLA): see the object, pick it up, put it down – arms + hands
   → Unitree's built-in walking controller
```

- **BotBrain** (approved for navigation): we use only its local parts, which are the map (RTAB-Map with the Mid-360), path planning (Nav2) and the G1 driver. I checked the code: that part makes no internet calls. Its web dashboard needs a cloud login (Supabase), and its AI chat needs OpenAI, so we leave both off. Its YOLO part has an AGPL license, so we leave that off too.
- **One program per body part:** BotBrain moves the legs, the VLA moves the arms and hands. If both send arm commands, the robot twitches.
- **The handover is the tricky part:** navigation stops roughly 10–20 cm from the ideal spot. So the training examples will vary the robot's start position, and the VLA learns to cope with that.

---

## Which VLA?

| | **π₀.₅ (openpi)** | **SmolVLA** |
|---|---|---|
| Quality | Stronger (≈3 B parameters) | Smaller (≈450 M), weaker on new situations |
| Training needs | NVIDIA GPU with ≥ 24 GB, so the cloud (AWS) | Runs on a MacBook overnight |
| Cost per training run | ≈ $30–60 (estimate, 30k steps on a rented H100) | free (own laptop) |

**Rule:** if AWS credits (or another big GPU) are confirmed **by 13 Oct**, we train π₀.₅. Otherwise SmolVLA. Both learn from the same recordings, so nothing is wasted. The trained model always **runs on the robot**, never in the cloud while the robot moves.

---

## Plan until Potsdam (18/19 Oct)

| Day | Work |
|---|---|
| **Wed 8 Oct** | More natural start pose · vary the robot's start position · add the waist to the recordings · **record 200–500 demos** · build the dataset · **start the first training overnight** |
| **Thu 9 Oct** | Test the trained VLA alone in the simulation → **Level 1 trained** |
| **Fri–Sun 10–12 Oct** | **Level 2:** realistic ball look, lighting, two balls + "put the blue ball in the basket" |
| **Mon 13 Oct** | Decide π₀.₅ vs SmolVLA (AWS?) |
| **Tue–Fri 14–17 Oct** | Real-robot connection, safety limits, tests, a step-by-step bring-up checklist · BotBrain installed locally on the robot (good task for a teammate) |
| **Sat/Sun 18/19 Oct** | **Potsdam:** careful step-by-step attempts on the real robot, film everything, record real demonstrations |

---

## Open questions (please help if you know)

1. **AWS credits:** yes or no, and by when? We need to know by ~13 Oct.
2. **Backpack PC:** which RTX card, how much GPU memory, which operating system (Linux?)
3. **BrainCo hands:** how are they wired, and which computer drives them?
4. **Potsdam:** robot time, a harness, a second person on the remote
5. Is turning the waist allowed through Unitree's arm interface (`rt/arm_sdk`)?
6. Do we have a VR headset for recording real demonstrations?
7. SAP legal: is the license of the π₀.₅ model weights (Gemma terms) OK?
