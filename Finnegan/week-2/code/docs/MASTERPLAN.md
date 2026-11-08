# Project masterplan: autonomous pick-and-place pipeline for the Unitree G1 EDU

**Virtual development environment, imitation learning and VLA control based on open-weight and open-source models**

---

## Contents

1. [Project goal & target scenario](#1-project-goal--target-scenario)
2. [Task decomposition of the target task](#2-task-decomposition-of-the-target-task)
3. [Hardware & robot specification](#3-hardware--robot-specification)
4. [Tech stack & architecture split](#4-tech-stack--architecture-split)
5. [System requirements & environment setup](#5-system-requirements--environment-setup)
6. [Project structure](#6-project-structure)
7. [Building the MuJoCo simulation scene](#7-building-the-mujoco-simulation-scene)
8. [Phase 1 – Data collection](#8-phase-1--data-collection)
9. [Phase 2 – Standardization in the LeRobot format](#9-phase-2--standardization-in-the-lerobot-format)
10. [Phase 3 – Fine-tuning with OpenPI / π₀ and LoRA](#10-phase-3--fine-tuning-with-openpi--π₀-and-lora)
11. [Phase 4 – Training run & monitoring](#11-phase-4--training-run--monitoring)
12. [Phase 5 – Closed-loop evaluation in simulation](#12-phase-5--closed-loop-evaluation-in-simulation)
13. [Orchestration: navigation + manipulation as a state machine](#13-orchestration-navigation--manipulation-as-a-state-machine)
14. [Sim-to-real transfer](#14-sim-to-real-transfer)
15. [Critical do's and don'ts](#15-critical-dos-and-donts)
16. [Milestone plan](#16-milestone-plan)
17. [Glossary](#17-glossary)

---

## 1. Project goal & target scenario

### 1.1 Overall goal

Build a complete, end-to-end **virtual development environment** for the humanoid robot **Unitree G1 EDU (29 DoF)**. The robot is trained fully in a physically correct simulation (**MuJoCo**). We use only **open-weight models** and **open-source frameworks**. Nothing is trained from scratch. We always build on existing foundation models (fine-tuning instead of pre-training).

### 1.2 Concrete target scenario

At the end of development, the Unitree G1 should do a **fully autonomous pick-and-place task**:

> **Voice command:** *"Go to the next room, get the red bottle from the table and bring it back."*

**Expected sequence without any human help:**

1. The robot navigates on its own into the next room.
2. It localizes the table and visually detects the red bottle on it.
3. It positions itself in front of the table (base placement).
4. It grasps the bottle with the dexterous hand (grasp).
5. It lifts the bottle and holds it steady in a carrying grasp.
6. It navigates back to the start position.
7. It places the bottle at the target position (place).

### 1.3 Core principle: hybrid architecture

We do **not** build **one monolithic model** that can do everything. Instead, there is a strict split of tasks:

| Layer | Responsibility | Technology |
|---|---|---|
| **High-Level** | Navigation, mapping, path finding, obstacle avoidance | Fast-LIO + Nav2 (LiDAR/SLAM) |
| **Mid-Level** | Task breakdown, state logic, handover between layers | State machine / behavior tree |
| **Low-Level** | Fine motor control, visual grasping, object manipulation | OpenPI / π₀ (VLA model) |

---

## 2. Task decomposition of the target task

The full task is split into subtasks. Each subtask can be trained and tested on its own. Each subtask is developed in isolation and only then chained to the others.

| # | Subtask | Responsible layer | Success criterion |
|---|---|---|---|
| T1 | Stable standing & balance | MuJoCo controller / GR00T policy | Robot does not tip over for 60 s |
| T2 | Walking to a target coordinate | Fast-LIO + Nav2 | Target point reached within ±10 cm |
| T3 | Room change through a door opening | Fast-LIO + Nav2 | Passes through without collision |
| T4 | Table and object detection | VLA (vision encoder) | Red bottle localized in the image |
| T5 | Approaching the grasp position | Nav2 + base placement heuristic | Bottle within reach of the arm |
| T6 | Grasping the bottle | OpenPI / π₀ | Stable grasp, no slipping |
| T7 | Lifting & carrying pose | OpenPI / π₀ | Bottle stays held for > 10 s |
| T8 | Navigating back with a load | Nav2 (adjusted center of mass) | Start point reached, load still held |
| T9 | Placing (place) | OpenPI / π₀ | Bottle stands upright, does not tip over |

> **Important:** T1–T3 and T8 are **classic robotics** (control, SLAM, planning). T4, T6, T7 and T9 are **learning tasks** for the VLA model. Do not mix these two kinds of tasks.

---

## 3. Hardware & robot specification

### 3.1 Robot platform

- **Model:** Unitree G1 EDU
- **Degrees of freedom:** 29 DoF (legs, torso, arms, wrists)
- **End effectors:** BrainCo Revo2 Dexterous Hands

### 3.2 Sensors & peripherals

| Component | Model | Function |
|---|---|---|
| Onboard camera | Intel RealSense D435i | Close range, depth sensing |
| Head camera module | Unitree G1-Head Dual-DoF Module with RealSense D455 | Main VLA input (RGB), can pan |
| LiDAR | Livox Mid360 | SLAM, localization, room navigation |

### 3.3 Onboard compute

| Unit | Specification | Role |
|---|---|---|
| Primary | NVIDIA Jetson Orin NX (16 GB) | Real-time control, sensor fusion, ROS2 |
| Backpack | Thunderobot computer, Intel Core i7 + NVIDIA GeForce RTX | VLA inference, Fast-LIO, compute-heavy nodes |

### 3.4 Development workstation (simulation & training)

- **CPU:** modern multi-core processor (at least 8 cores recommended)
- **GPU:** NVIDIA RTX 4090 (24 GB VRAM) or a cloud A100
- **RAM:** at least 32 GB, 64 GB recommended
- **Storage:** SSD with at least 500 GB of free space (datasets + checkpoints)

---

## 4. Tech stack & architecture split

### 4.1 Component overview

| Component | Purpose | License model |
|---|---|---|
| **MuJoCo** | Physics engine, simulation core. Loads the G1 MJCF model incl. hands and head camera. | Open Source |
| **Fast-LIO** | LiDAR-inertial odometry. Turns Livox Mid360 point clouds into a map + localization. | Open Source |
| **Nav2 (ROS2)** | Path planning, cost maps, obstacle avoidance. | Open Source |
| **LeRobot** (Hugging Face) | Standard format for robotics datasets. Stores images, voice commands and proprioception in synchronized episodes. | Open Source |
| **OpenPI / π₀** | Vision-language-action foundation model. Controls the arms and hands. | Open Weights |
| **NVIDIA Isaac GR00T** | Higher-level framework for robot policies and control logic. | Open Model |
| **uv** (Astral) | Python package management and environment isolation. | Open Source |
| **Weights & Biases** | Training monitoring. | SaaS / open alternative: TensorBoard |

### 4.2 Data flow at run time

```
                    ┌──────────────────────────┐
Voice command ────► │   Task Orchestrator      │
                    │   (State Machine)        │
                    └────┬────────────────┬────┘
                         │                │
            NAVIGATION   │                │   MANIPULATION
                         ▼                ▼
          ┌──────────────────────┐   ┌─────────────────────────┐
Livox ───►│ Fast-LIO → Nav2      │   │ OpenPI / π₀ (VLA)       │◄─── RGB (D455)
Mid360    │ Map, path, twist     │   │ Image + text → actions  │◄─── Text command
          └──────────┬───────────┘   └───────────┬─────────────┘
                     │ Base velocity             │ Action chunks (arm/hand)
                     ▼                           ▼
          ┌────────────────────────────────────────────────────┐
          │  Low-level controller / MuJoCo simulation (G1)     │
          └────────────────────────────────────────────────────┘
```

### 4.3 Why this split?

- **Avoid overload:** A single model for navigation *and* fine motor control needs many times more data. In practice, it does not converge.
- **Easier debugging:** If grasping fails, we do not need to retrain navigation.
- **Avoid build errors:** ROS2/Fast-LIO (C++) and the Python VLA pipeline run in separate environments. They talk to each other over defined interfaces.

---

## 5. System requirements & environment setup

### 5.1 Operating system

**Required:** native **Ubuntu 22.04 LTS** or a fast **WSL2 with graphics acceleration**.

> ⚠️ **No native Windows development.** The tech stack is based on C++ libraries and on ROS2 and Fast-LIO builds. These do not work reliably on Windows.

### 5.2 Python version

Python **3.10** or **3.11**. Package management only with **`uv`** (Astral), for fast, clean and reproducible environment isolation.

### 5.3 Step-by-step installation

#### Step 1 – System updates and build tools

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git curl \
    libglfw3 libglfw3-dev libgl1-mesa-dev libosmesa6-dev \
    python3-dev
```

#### Step 2 – Install `uv` and create the project folder

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc

mkdir -p ~/g1_vla_project && cd ~/g1_vla_project
```

#### Step 3 – Clone the repositories

```bash
cd ~/g1_vla_project
git clone https://github.com/huggingface/lerobot.git
git clone https://github.com/Physical-Intelligence/openpi.git
```

#### Step 4 – Create a virtual environment and install the core packages

```bash
cd ~/g1_vla_project
uv venv --python 3.11
source .venv/bin/activate

uv pip install mujoco numpy scipy matplotlib opencv-python
uv pip install h5py zarr tqdm pyyaml wandb
```

#### Step 5 – Add the robot model

Put the Unitree G1 URDF/MJCF model files, including the hands and the head camera, into this folder:

```
~/g1_vla_project/assets/g1_description/
```

#### Step 6 – Check the installation

```bash
python -c "import mujoco; print('MuJoCo', mujoco.__version__)"
python -m mujoco.viewer   # opens the viewer to check the graphics acceleration
```

#### Step 7 – Set up a minimal test scene

Write a short Python script with a main loop (`mujoco.viewer.launch_viewer`). It only checks whether the robot stands **physically stable**. No objects, no textures, no cameras yet.

```python
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("assets/g1_description/scene.xml")
data  = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

> **Exit criterion for this phase:** The robot stands for 60 seconds without drift, without shaking and without sinking into the floor. Only then move on.

---

## 6. Project structure

```
~/g1_vla_project/
├── .venv/                          # uv environment
├── assets/
│   ├── g1_description/             # URDF/MJCF of the Unitree G1 (incl. hands)
│   ├── objects/                    # bottle, table, room meshes
│   └── scenes/
│       ├── scene_minimal.xml       # stability test
│       ├── scene_table_grasp.xml   # manipulation training scene
│       └── scene_two_rooms.xml     # full target scenario
├── lerobot/                        # cloned repository
├── openpi/                         # cloned repository
│   └── configs/
│       └── g1_finetune_grasp.yaml  # our own training config
├── scripts/
│   ├── collect_demos.py            # teleoperation & recording
│   ├── convert_to_lerobot.py       # conversion + validation
│   ├── validate_dataset.py         # timestamp and integrity check
│   ├── train.py                    # training entry point
│   ├── eval_closed_loop.py         # inference test in MuJoCo
│   └── orchestrator.py             # state machine navigation ↔ manipulation
├── datasets/
│   └── g1_bottle_pick_v1/          # HDF5/Zarr episodes
├── checkpoints/
└── README.md
```

---

## 7. Building the MuJoCo simulation scene

### 7.1 Build order (follow it strictly)

1. **`scene_minimal.xml`** – only floor + G1. Goal: stability, joint limits, damping.
2. **`scene_table_grasp.xml`** – floor + G1 + table + red bottle. Goal: grasp training.
3. **`scene_two_rooms.xml`** – two rooms with a door opening, table in the target room. Goal: full task.

### 7.2 Physical parameters that must be right

| Parameter | Meaning | Note |
|---|---|---|
| `timestep` | Integration step | 0.002 s as a start value |
| `solver` | Constraint solver | Newton for stable contacts |
| `friction` | Friction hand ↔ bottle | Too low = the object always slips |
| `condim` | Contact dimensions | 4 or 6 for grasp contacts |
| `mass` / `inertia` | Mass distribution | Realistic bottle mass (~0.5 kg when full) |
| `armature` | Rotor inertia of the joints | Prevents numerical instability |

### 7.3 Virtual camera

The virtual head camera must be defined **at exactly the position and with the FOV of the real RealSense D455**. Differences here create a sim-to-real gap that is very hard to close later.

```xml
<camera name="head_cam" mode="fixed" pos="0 0.08 0.62"
        euler="0 0 0" fovy="58"/>
```

Recording resolution: **224×224** or **256×256** (VLA input format), recording rate **30 FPS**.

---

## 8. Phase 1 – Data collection

### 8.1 Principle: imitation learning

The robot does **not** learn through a reward system (reinforcement learning). It learns by **exactly imitating** demonstrated solutions. The quality of the demonstrations fully decides the quality of the end result.

### 8.2 How it is done

The virtual Unitree G1 in MuJoCo is controlled with:

- **Gamepad** (recommended for smooth, natural movements),
- **Keyboard** (only for simple movements),
- **Python control script** (scripts reproducible trajectories, e.g. via inverse kinematics).

Task per episode: *walk to the table → see the red bottle → grasp → lift.*

### 8.3 Streams to record (synchronized)

| Stream | Content | Rate |
|---|---|---|
| **Visual** | RGB image series of the virtual RealSense head camera | 30 FPS |
| **Language** | Text command as a string, e.g. *"Grasp the red bottle"* | 1× per episode |
| **Kinematics (proprioception)** | Exact joint angles and joint velocities of all 29 DoF plus the hand joints | 30 Hz, in sync with the image |
| **Action** | Commanded target joint angles (the learning target) | 30 Hz |

### 8.4 Target amount

- **50 to 100 clean, error-free episodes** at the start.
- Episode length: **5 to 15 seconds**, depending on the task.
- Later, targeted additions of **10–20 episodes per identified failure case**.

### 8.5 Domain randomization during recording

Vary these already during data collection:

- Table color and texture
- Light intensity, color and direction
- Start position and rotation of the bottle (± 15 cm, ± 30°)
- Small changes of the camera angle
- Distractor objects on the table (other colors, so that "red" is the feature that tells them apart)

### 8.6 Frame rate requirement

Simulation **and** sensor recording must run stably at **at least 30 FPS**. Frame drops create asynchronous datasets, and then the training data are useless.

---

## 9. Phase 2 – Standardization in the LeRobot format

OpenPI needs the data in a strictly defined format that is compatible with the Hugging Face **LeRobot** standard.

### 9.1 Structuring

The data are written into standard containers: **HDF5** or **Zarr**.

Typical episode structure:

```
episode_0001/
├── observation.images.head_cam    # (T, 224, 224, 3) uint8
├── observation.state              # (T, 29+N_hand) float32
├── action                         # (T, 29+N_hand) float32
├── task                           # "Grasp the red bottle"
├── timestamp                      # (T,) float64
└── episode_index / frame_index
```

### 9.2 Validation

A validation script (`scripts/validate_dataset.py`) must run before every training run. It checks:

- [ ] The timestamps of the camera images match the joint actions (action chunks) **exactly**.
- [ ] No lost or duplicated frames.
- [ ] No asynchronous sensor data (drift between the image stream and the state stream).
- [ ] No NaN/Inf values in state or action vectors.
- [ ] Every episode has a text command.
- [ ] Normalization statistics (mean/std) are computed and saved.

> **Rule:** A dataset that fails the validation is not used for training. Errors in the data cost many times more time in training.

---

## 10. Phase 3 – Fine-tuning with OpenPI / π₀ and LoRA

**π₀** is a very large foundation model. So we do **not train it fully from zero**. Instead, we **fine-tune** the pretrained weights.

### 10.1 Create the config file

A new YAML file in the OpenPI repository, e.g. `configs/g1_finetune_grasp.yaml`.

### 10.2 Important parameters

```yaml
# configs/g1_finetune_grasp.yaml

model:
  base_checkpoint: /path/to/pi0_base_checkpoint     # pretrained π₀ base model
  action_horizon: 32                                # action chunk length (20–50)

data:
  dataset_path: ~/g1_vla_project/datasets/g1_bottle_pick_v1
  train_split: 0.9
  val_split: 0.1
  image_keys: ["observation.images.head_cam"]
  state_key: "observation.state"
  action_key: "action"

lora:
  enabled: true            # train ONLY the action expert + adapters
  rank: 16                 # start value; 8–32 is common
  alpha: 32
  dropout: 0.05
  target_modules: ["q_proj", "k_proj", "v_proj", "o_proj"]

training:
  batch_size: 8            # adjust to the VRAM
  learning_rate: 1.0e-5    # very low on purpose
  warmup_steps: 500
  num_epochs: 30
  gradient_accumulation_steps: 4
  checkpoint_every_n_epochs: 2
  seed: 42

logging:
  wandb_project: g1_vla_pick_place
```

### 10.3 Why LoRA?

**LoRA (low-rank adaptation)** trains only the **action expert** and small adapter matrices, not all weights. This saves a lot of VRAM. It makes training possible on a **single RTX 4090** or on a cloud GPU. The pretrained world and language representations of the base model stay intact.

---

## 11. Phase 4 – Training run & monitoring

### 11.1 Start in the terminal

```bash
cd ~/g1_vla_project/openpi
uv run scripts/train.py g1_finetune_grasp --exp-name=g1_mujoco_run1
```

### 11.2 Monitoring

Connect the training to **Weights & Biases (W&B)**.

Values to watch:

| Metric | Expected behavior | Warning sign |
|---|---|---|
| **Training Loss** | Falls steadily over the epochs | Plateau in the first epochs → check the learning rate or the data |
| **Validation Loss** | Falls together with the training loss | Splits off and rises → **overfitting** |
| **Gradient Norm** | Stable in a narrow band | Spikes → learning rate too high |
| **Action MSE** | Falls | No progress → dataset too small or inconsistent |
| **GPU load / VRAM** | Constantly high, without OOM | OOM → lower the batch size or the LoRA rank |

### 11.3 Checkpointing

The training script must save a checkpoint **every few epochs**. Reasons:

- Process crash (OOM, power cut, cloud timeout)
- Overfitting – the best checkpoint is often not the last one
- Catastrophic forgetting – going back to an earlier, intact state

---

## 12. Phase 5 – Closed-loop evaluation in simulation

After training, there is a new model checkpoint. Now we check whether the model can act **on its own**.

### 12.1 Start the inference loop

A test script (`scripts/eval_closed_loop.py`) opens the MuJoCo simulator and hands control to the trained model.

### 12.2 Autonomous operation

The VLA model gets **only**:

- the current RGB image of the head camera,
- the text command,
- optionally the proprioceptive state vector.

From these, it computes the next joint angles live, as **action chunks**. No ground-truth positions, no hidden states from the simulator.

```python
while not done:
    obs = {
        "image": render_head_cam(model, data),
        "state": data.qpos[joint_ids].copy(),
        "task":  "Grasp the red bottle",
    }
    action_chunk = policy.infer(obs)          # e.g. (32, n_joints)
    for action in action_chunk:
        data.ctrl[:] = action
        mujoco.mj_step(model, data)
```

### 12.3 Failure analysis

Log these cases systematically:

- Does the robot grasp **next to** the object? → vision problem or camera calibration error
- Does it **drop the object**? → friction parameters, or too few holding demos
- Does it **jerk** nervously? → action chunk length too short
- Does it **not move at all**? → normalization statistics or action space mapped wrong
- Does it only work for **one** bottle position? → overfitting, domain randomization is missing

### 12.4 Iterative approach

For every situation where the robot fails: record **10–20 new demos for exactly this case** (data augmentation / corner cases), add them to the dataset and train again for a short time. Repeat this cycle several times.

**Success metric:** success rate over at least 50 randomized runs. Target value before sim-to-real: **> 85 %**.

---

## 13. Orchestration: navigation + manipulation as a state machine

For the full task ("get the red bottle from the next room"), an orchestrator connects the two layers. It decides **when** which layer has control. It never steps into the control loop itself.

```
 ┌──────────┐
 │   IDLE   │  Waits for a voice command
 └────┬─────┘
      │ Command received → task parsing (goal, object, return place)
      ▼
 ┌──────────────────┐
 │ NAVIGATE_TO_ROOM │  Fast-LIO + Nav2 active, VLA inactive
 └────┬─────────────┘
      │ Target room reached
      ▼
 ┌──────────────────┐
 │  LOCATE_OBJECT   │  Head camera scans, VLA/detector looks for the red bottle
 └────┬─────────────┘
      │ Object found
      ▼
 ┌──────────────────┐
 │ APPROACH_TABLE   │  Nav2 drives to the grasp position (base placement)
 └────┬─────────────┘
      │ Bottle within reach
      ▼
 ┌──────────────────┐
 │      GRASP       │  ◄── VLA (OpenPI/π₀) takes over fully
 └────┬─────────────┘
      │ Grasp stable (force/contact check)
      ▼
 ┌──────────────────┐
 │   LIFT_AND_HOLD  │  ◄── VLA, carrying pose
 └────┬─────────────┘
      │ Object secured
      ▼
 ┌──────────────────┐
 │  NAVIGATE_BACK   │  Nav2, center of mass adjusted for the load
 └────┬─────────────┘
      │ Start position reached
      ▼
 ┌──────────────────┐
 │      PLACE       │  ◄── VLA, placing
 └────┬─────────────┘
      ▼
 ┌──────────┐
 │   DONE   │
 └──────────┘
```

### 13.1 Handover points (critical)

| Transition | Condition | Fallback on failure |
|---|---|---|
| Nav2 → VLA | Object in the image + distance < arm reach | Reposition, try again |
| VLA → Nav2 | Contact force at the hand > threshold for 1 s | Release the grasp, grasp again |
| Carrying | No loss of contact | E-stop, secure the object |

> **Rule:** Never let both layers command joints of the same chain link at the same time. Each state has clear authority over a defined set of joints.

---

## 14. Sim-to-real transfer

### 14.1 Requirements before the first real trial

- [ ] Success rate in simulation > 85 % over randomized runs
- [ ] Domain randomization active for light, colors, textures and object positions
- [ ] Virtual camera parameters match the real D455 (FOV, resolution, position)
- [ ] Joint limits and torque limits in the simulation match the real G1
- [ ] E-stop procedure tested and within reach
- [ ] Control frequency on the Jetson Orin NX / backpack measured and high enough

### 14.2 Approach

1. **Measure the inference latency:** Run the VLA model on the backpack and check the real rate. Action chunks buffer the latency. This is one of their main advantages.
2. **First test without a load:** Movement sequence in the air, without an object, with reduced torque.
3. **Static grasp test:** The robot stands fixed and grasps at a fixed bottle position.
4. **Full task under supervision:** A person with the e-stop stays within reach at all times.
5. **Add real data:** Record real demos and mix them into the dataset (co-training sim + real).

---

## 15. Critical do's and don'ts

### ✅ DO's – always do these

**Architecture**

- **Keep the layers separate.** Fast-LIO controls the room navigation. The fine motor control at the table belongs fully to the OpenPI VLA model. No monolithic model for everything.
- **Use action chunks.** Configure OpenPI so that the model outputs short movement sequences (the next 20–50 time steps) at once, not only the next micro step. This prevents nervous jerking.

**Simulation & data**

- **Always use domain randomization.** Vary table colors, lighting, object positions and start poses regularly, during data collection *and* in training. This is the only way to close the sim-to-real gap.
- **Keep the frame rate stable.** Simulation and sensor recording must run stably at **30 FPS** or more.
- **Record clean demos.** Stop failed simulation runs at once. Imitation learning also learns the errors, and this causes uncontrolled jerking.

**Training**

- **Save checkpoints regularly.** Every few epochs, as protection against process crashes, overfitting and catastrophic forgetting.
- **Use a validation split.** Split the data, e.g. 90 % training / 10 % validation. This checks whether the model understood the task or only learned the demos by heart.
- **Physics before visuals.** First set stability, collisions and friction correctly, then everything else.

### ❌ DON'Ts – strictly avoid

**Environment**

- **Never develop on Windows.** No native Windows installs for this tech stack; always use Ubuntu 22.04 (Linux).

**Training**

- **No full fine-tuning of π₀.** Never try to train the whole base model from scratch. Use LoRA instead.
- **No learning rates that are too high.** When you fine-tune a pretrained VLA model, a learning rate that is too high destroys the existing world and language patterns ("catastrophic unlearning"). Guide value: `1e-5`.
- **Do not train without a validation split.** Without a split, you cannot see overfitting.

**Data**

- **Never keep unclean demos.** If the robot stumbled in episode 12 or missed the object, delete this episode at once. Imitation learning copies inefficiencies and errors one to one.
- **No rigid pixel expectations.** Without domain randomization, the model will fail in the real world.

**Simulation**

- **No photorealistic ray-tracing scenes at the start.** Do not overload the simulation with graphics extras. Physical stability, correct collisions and friction are essential; shadows are not.

---

## 16. Milestone plan

| M | Milestone | Acceptance criterion | Depends on |
|---|---|---|---|
| **M0** | Environment runs | `mujoco.viewer` starts, G1 model loads without errors | – |
| **M1** | Physical stability | G1 stands for 60 s without drift or sinking into the floor | M0 |
| **M2** | Manipulation scene | Table + red bottle + virtual head camera at 30 FPS | M1 |
| **M3** | Teleoperation pipeline | An episode can be recorded, all streams in sync | M2 |
| **M4** | Dataset v1 | 50–100 clean episodes, validation passed | M3 |
| **M5** | First fine-tuning | Training converges, val loss falls | M4 |
| **M6** | Closed-loop grasping | Success rate > 60 % with a fixed bottle position | M5 |
| **M7** | Robust grasping | Success rate > 85 % with randomized position/light | M6 |
| **M8** | Navigation in simulation | Room change with Fast-LIO/Nav2 without collision | M2 |
| **M9** | Orchestrated full task | Complete pick-and-place cycle in simulation | M7 + M8 |
| **M10** | Sim-to-real preparation | Latency measurement on the target hardware, safety concept | M9 |
| **M11** | Real test run | Task under supervision on the physical G1 | M10 |

---

## 17. Glossary

| Term | Meaning |
|---|---|
| **VLA** | Vision-language-action model. Takes an image + a voice command and directly outputs robot actions. |
| **π₀ (Pi-Zero)** | Open-weight VLA foundation model, provided through OpenPI. |
| **Imitation Learning** | Learning method that imitates demonstrated solutions instead of using reward signals. |
| **LoRA** | Low-rank adaptation. Fine-tuning method that trains only small adapter matrices and so saves VRAM. |
| **Action Chunk** | A connected sequence of several future actions that the model outputs in one step. |
| **Proprioception** | The robot's sense of its own body: joint angles, velocities, forces. |
| **Domain Randomization** | Systematic variation of simulation parameters, to create robust models that transfer well. |
| **Sim-to-Real Gap** | Loss of performance when a model trained in simulation is moved to real hardware. |
| **Overfitting** | The model learns the training examples by heart instead of the underlying task. |
| **Catastrophic Unlearning** | Loss of pretrained knowledge through fine-tuning that is too aggressive. |
| **MJCF** | MuJoCo XML Format – the description format for models and scenes. |
| **Fast-LIO** | LiDAR-inertial odometry method for mapping and localization. |
| **SLAM** | Simultaneous Localization and Mapping. |
| **DoF** | Degree of Freedom. |

---

*This document is a technical reference and a development guide. All parameter values are start values. They must be tuned by experiment during the project.*

---

## Appendix A — Status & corrections (2026-09-14)

> Added when the plan was moved into the repo (today `First Ideas/UnitreeG1-Robot`). **Everything above this appendix is unchanged in content.
> It was translated to English on 2026-10-07** (German original: `Documents/…/First Ideas/G1_VLA_Masterplan.md`). The current checklist is in
> `PROGRESS.md` → NEXT STEP, the reasons for the scope are in `PLAN.md` → SCOPE UPDATE 2026-09-14.

### A.1 Status per milestone

Numbers measured on 2026-09-14, not copied from the docs.

| M | Criterion (§16) | Status | Evidence / what is missing |
|---|---|---|---|
| M0 | Environment runs | ✅ | mujoco 3.12, viewer confirmed. macOS (M4) + Python 3.12 on purpose, instead of Ubuntu/uv |
| M1 | 60 s stable | 🟡 | **Fixed base passed:** 30,000 steps, 0 solver warnings, bottle drift 0.22 cm. Free standing: no, but it is not needed (walking controller, see A.3 no. 7) |
| M2 | Table + bottle + head camera | 🟡 | Table, red bottle, 2 distractors, head camera + 2 wrist cameras, Revo2 hands ✅. Missing: hand mount not verified · only the right hand has `grasp_site` · bottle 0.35 kg · `condim` 3 · no torque limits (`forcerange` 0) · ~~camera fixed on the torso instead of the D455 head module~~ (2026-09-16: `head_camera` = D455 on the head module, `g1sim/payload.py`) |
| M3 | Teleoperation pipeline | ❌ | no recorder. The web control panel (`dashboard/`) has sliders, but it is not teleop |
| M4 | Dataset v1 | ❌ | no writer, no validator |
| M5 | First fine-tuning | ❌ | no training setup, no Linux computer with a GPU |
| M6/M7 | Closed-loop grasping | ❌ | missing: VLM in the loop, IK, eval harness |
| M8 | Navigation in simulation | ❌ | no walking sim, no two-room scene |
| M9 | Orchestrated full task | ❌ | no orchestrator |
| M10/M11 | Sim2Real | ❌ | driver interface `dashboard/drivers.py` exists, DDS driver is missing |

**Already built, but not in this plan:** pixel→3D grasp point with 0.9 cm error
(`scripts/13_grasp_target.py`), web control panel with a hand mount that can be adjusted live, `docs/DATA_STRATEGY.md`
(sim as a demo factory + co-training with real data).

### A.2 Project structure §6 → repo

| §6 | Repo | Status |
|---|---|---|
| `~/g1_vla_project/` | `First Ideas/UnitreeG1-Robot/` (since 2026-09-15; ROS 2 runs on the Linux computer, where the path has no spaces) | ✅ |
| `assets/g1_description/` | Menagerie `unitree_g1` + `models/revo2/` | ✅ |
| `scenes/scene_minimal.xml` | Menagerie G1 (`scripts/00_view_g1.py`) | ✅ |
| `scenes/scene_table_grasp.xml` | `models/g1_brainco/scene_table_hands.xml`, built by `dashboard/scene.py::build_with_hands` | ✅ (physics still needs work) |
| `scenes/scene_two_rooms.xml` | — | ❌ |
| `scripts/collect_demos.py` | — | ❌ |
| `scripts/convert_to_lerobot.py`, `validate_dataset.py` | — | ❌ |
| `openpi/configs/g1_finetune_grasp.yaml` | — (wrong format, see A.3 no. 1) | ❌ |
| `scripts/eval_closed_loop.py` | — | ❌ |
| `scripts/orchestrator.py` | — | ❌ |
| *missing in §6* | action space module (26-D), IK, VLM client, domain randomization, safety guard | ❌ |

### A.3 Corrections

| # | § | Masterplan says | What is correct | Source |
|---|---|---|---|---|
| 1 | §10.1, §10.2 | YAML config, LoRA with `target_modules`, "only action expert", epochs | openpi configs are **Python dataclasses** (`TrainConfig` in `src/openpi/training/config.py`). LoRA = `paligemma_variant="gemma_2b_lora"` + `action_expert_variant="gemma_300m_lora"` + `freeze_filter`, so it is on **both** parts. Training is counted in steps (defaults: peak_lr 5e-5 cosine, warmup 10,000, batch 32, 30,000 steps, save_interval 1000). With LeRobot π₀.₅ these are CLI flags; `train_expert_only=true` saves memory | openpi `config.py`; LeRobot π₀.₅ docs |
| 2 | §9.1 | HDF5 or Zarr | LeRobot datasets are **Parquet + MP4**. openpi pins lerobot commit `0cf8648` = format **v2.1**; current LeRobot writes **v3.0**. Never mix them | openpi `pyproject.toml` |
| 3 | §8.3, §9.1 | State/action `29 + N_hand` | Action = **26-D `g1_brainco`**: 7 arm joints per side (incl. wrist pitch/yaw) + 6 hand joints per side. The legs belong to the walking controller. **Same vector for 23 and 29 DoF** (with 23 DoF there are four phantom slots). Hand normalized 0…1, **0 = open, 1 = closed**, order thumb, ThumbAux, index, middle, ring, pinky; real ranges 1.52 / 1.05 / 1.47 rad (sim: 1.51 / 1.04 / 1.46) | `unitree_lerobot` `Unitree_G1_Brainco`; `xr_teleoperate` `robot_hand_brainco.py` |
| 4 | §8.6 | below 30 FPS the data become useless | Only true for live teleop. Scripted recording renders in a paused sim → in sync by design; we record at 30 Hz **sim time**. Measured: 3 cameras at 224×224 = 18.5 fps live | Measurement 2026-09-14 |
| 5 | §7.3 | `head_cam` at `0 0.08 0.62`, fovy 58 | Placeholder. The real camera sits on the D455 head module, which **can pan**. **Done since 2026-09-16:** head module from Unitree's `g1_comp.urdf` (servo XL330 pans ±50°, D455 tilts −20…+90°), `head_camera` = D455 color camera following Intel's `realsense-ros`, fovy 65 (RGB 90×65°); tilted 0.85 rad at the table (`g1sim/payload.py`). Open: image format 16:10 instead of square, calibrate the lens height on the robot. Camera names in LeRobot/Unitree: `cam_left_high`, `cam_right_high`, `cam_left_wrist`, `cam_right_wrist` | `scene_table_hands.xml`; `unitree_lerobot` |
| 6 | §8.2 | Gamepad/keyboard teleop | Not practical for 26 DoF. Create demos **scripted via IK** (`docs/DATA_STRATEGY.md` §3.1); real teleop later with `xr_teleoperate` | — |
| 7 | §2 T1, M1 | Standing with a MuJoCo controller / GR00T policy, 60 s free | Standing and walking (leg motion + balance) are done by a **ready-made walking controller**. This is not remote control: the commands come autonomously from Nav2. In the sim: grasping with a fixed base (passed), walking with the LeRobot controller (no. 10) | Measurement 2026-09-14 |
| 8 | §5.1, §15 | only Ubuntu 22.04 | True for openpi, `xr_teleoperate`, ROS 2, Fast-LIO. MuJoCo sim, scene building and data generation are proven to run on macOS | `PROGRESS.md` Phase 0 |
| 9 | §2 T2/T3, M8 | Fast-LIO + Nav2 in simulation | Not on macOS, but possible in the sim: simulate the Livox Mid360 + IMU in MuJoCo → Fast-LIO + Nav2 on a Linux box against the sim. Step before that: navigation with a known pose, same interface (vx, vy, ω). The same stack later runs on the robot | — |
| 10 | §4.1, §5, §11 | own stack from MuJoCo + LeRobot + openpi | **LeRobot v0.5 (March 2026) comes with a ready-made G1 integration:** MuJoCo sim `lerobot/unitree-g1-mujoco` (29 DoF, DDS, torque limits, elastic band safety), walking controller `GrootLocomotionController` / `HolosomaLocomotionController`, π₀.₅ training, RTC inference. Sim and real via `--robot.is_simulation`. Hands there: Dex3, not BrainCo. Rejected: `unitree_rl_gym` `motion.pt` (trained on `g1_12dof.xml` without arms) | LeRobot docs "Unitree G1" |
| 11 | §7.2 | condim 4–6, bottle ~0.5 kg | Not done yet: condim 3, bottle 0.35 kg, no torque limits. Target values for the limits: legs 88/88/88/139/50/50 Nm, waist 88/50/50, arms 25, wrist pitch/yaw 5 Nm | Measurement 2026-09-14; `lerobot/unitree-g1-mujoco` `config.yaml` |
| 12 | §5.2, §5.3 | Python 3.10/3.11, `uv` | The repo runs on Python 3.12.14 (venv + pip). LeRobot G1 uses Python 3.12; openpi needs ≥ 3.11 (Linux only) | `README.md`; LeRobot docs |

### A.4 Build order

Masterplan = the goal, built in stages. Each stage ends with a demo that runs.

| Stage | Content | Milestone |
|---|---|---|
| 0 | Spike: LeRobot G1 sim on the Mac · one shared MJCF base · training stack · sort out Linux+GPU | — |
| 1 | Grasping at the table without training (VLM → 3D → IK) + eval harness | M2 |
| 2 | π₀.₅ learns grasping from sim demos of Stage 1 | M3–M7 |
| 3 | Autonomous walking + navigation: ready-made walking controller for the legs; our own navigation, first with a known pose, then simulated LiDAR → Fast-LIO + Nav2 | M8 |
| 4 | Orchestrator (state machine §13) | M9 |
| 5 | Sim2Real | M10–M11 |

**Why grasping first:** Only grasping is trained. The training data come from a
working grasp pipeline, and the leg control already exists. Stage 0 comes before that,
so that the grasp scene is not built twice, on two different G1 models.

### A.5 Sources (checked 2026-09-14)

- openpi: https://github.com/Physical-Intelligence/openpi (`pyproject.toml`, `src/openpi/training/config.py`)
- LeRobot Unitree G1: https://huggingface.co/docs/lerobot/unitree_g1
- LeRobot π₀.₅: https://huggingface.co/docs/lerobot/pi05
- LeRobot G1 sim: https://huggingface.co/lerobot/unitree-g1-mujoco
- unitree_lerobot: https://github.com/unitreerobotics/unitree_lerobot
- xr_teleoperate: https://github.com/unitreerobotics/xr_teleoperate
- mink (IK): https://github.com/kevinzakka/mink
- unitree_rl_gym: https://github.com/unitreerobotics/unitree_rl_gym
- G1 arms while walking (`rt/arm_sdk`): https://github.com/unitreerobotics/unitree_sdk2_python/issues/146
