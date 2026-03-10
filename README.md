# SAC-Augmented PID Control for Quadcopter UAVs

**AI-Driven Adaptive PID Tuning via Soft Actor-Critic Reinforcement Learning**

![MATLAB](https://img.shields.io/badge/MATLAB-R2024b+-E16737?logo=mathworks&logoColor=white)
![Simulink](https://img.shields.io/badge/Simulink-Aerospace_Blockset-0076A8?logo=mathworks&logoColor=white)
![RL Toolbox](https://img.shields.io/badge/Reinforcement_Learning-Toolbox-00B2A9)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

> A hybrid control system that augments classical cascaded PID with a Soft Actor-Critic (SAC) agent, achieving up to **71% RMSE improvement** in position tracking under wind disturbances.

---

## Table of Contents

- [Problem Statement](#problem-statement)
- [Proposed Solution](#proposed-solution)
- [System Architecture](#system-architecture)
- [Key Results](#key-results)
- [Project Structure](#project-structure)
- [Requirements](#requirements)
- [Quick Start](#quick-start)
- [Project Phases](#project-phases)
- [Technical Details](#technical-details)
- [Visualization](#visualization)
- [License](#license)

---

## Problem Statement

Classical cascaded PID controllers achieve precise position tracking for quadcopter UAVs in calm conditions. However, **fixed PID gains** represent an inherent compromise between responsiveness and stability — they **cannot adapt** to changing environmental conditions such as:

- Sudden crosswind gusts
- Sustained sinusoidal wind patterns
- Stochastic atmospheric turbulence

This results in significant performance degradation (up to **0.41 m RMSE**) when the quadcopter encounters external disturbances.

---

## Proposed Solution

A **Soft Actor-Critic (SAC)** reinforcement learning agent acts as a supervisory layer that dynamically adjusts the inner PID loop's gain parameters (Kp, Ki, Kd) in real time. The agent **does not replace** the PID controller — it **continuously tunes** it based on observed tracking errors and current system state.

### Why SAC?

| Feature | Benefit |
|---|---|
| **Off-policy** | Sample-efficient; reuses past experience via replay buffer |
| **Entropy regularisation** | Naturally explores the gain space; avoids premature convergence |
| **Continuous actions** | Directly outputs smooth gain adjustments without discretisation artifacts |
| **Twin critic architecture** | Prevents Q-value overestimation, ensuring stable training |

---

## System Architecture

```
                    ┌─────────────┐
  Setpoint ────────►│  Outer PID  │───► desired φ, θ
  [x, y, z]        │  (50 Hz)    │
                    └─────────────┘
                           │
                    ┌──────▼──────┐    ┌────────────┐
                    │  Inner PID  │◄───│  SAC Agent  │
                    │  (500 Hz)   │    │  (50 Hz)    │
                    │  φ, θ, ψ    │    │  ΔKp,Ki,Kd  │
                    └──────┬──────┘    └──────▲──────┘
                           │                  │
                    ┌──────▼──────┐    15-dim observation:
                    │  Thrust     │    [pos err, att err,
                    │  Mixer      │     vel err, curr gains]
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐    ┌─────────────┐
                    │  6-DOF      │◄───│    Wind      │
                    │  Plant      │    │  Disturbance │
                    └─────────────┘    └─────────────┘
```

The SAC agent observes a **15-dimensional state vector** (position errors, attitude errors, velocity errors, angular rates, and current effective gains) and outputs **6 gain deltas** that adjust the roll and pitch inner-loop PID parameters in real time.

---

## Key Results

| Scenario | Baseline PID RMSE | AI-Augmented PID RMSE | Improvement |
|:---|:---:|:---:|:---:|
| Calm air | 0.002 m | 0.002 m | — |
| Step wind (5 N) | 0.24 m | 0.07 m | **71%** |
| Sinusoidal wind (2 Hz, 3 N) | 0.31 m | 0.11 m | **65%** |
| Stochastic turbulence | 0.41 m | 0.14 m | **66%** |

> The SAC-augmented controller matches baseline performance in calm conditions while dramatically reducing tracking error under all disturbance scenarios.

---

## Project Structure

```
Quadcopter_control_system/
├── setup.m                         # Master entry point — loads params & creates agent
├── README.md
│
├── scripts/
│   ├── init_params.m               # Physical constants, PID gains, simulation parameters
│   ├── create_sac_agent.m          # SAC agent with actor/critic neural networks
│   ├── build_simulink_model.m      # Programmatic Simulink model builder
│   ├── train_agent.m               # RL training loop with domain randomisation
│   ├── reward_function.m           # Reward logic with built-in unit tests
│   ├── evaluate_and_plot.m         # Metrics computation and figure generation
│   └── visualize_flight.m          # 3D animated quadcopter flight demo
│
├── models/
│   ├── quadcopter_rl_env.slx       # Main Simulink model (RL environment)
│   ├── quadcopter_baseline_pid.slx # PID-only baseline model
│   └── subsystems/
│       └── plant_6dof_notes.md     # 6-DOF plant modelling notes
│
├── docs/
│   ├── architecture_overview.md    # Detailed system design narrative
│   └── simulink_build_guide.md     # Step-by-step Simulink wiring guide
│
├── saved_agents/                   # Trained agent checkpoints (.mat files)
└── results/                        # Generated figures and evaluation outputs
```

---

## Requirements

- **MATLAB R2024b** or later
- **Simulink**
- **Aerospace Blockset** — 6-DOF rigid body dynamics
- **Reinforcement Learning Toolbox** — SAC agent, training, environment
- **Control System Toolbox** — PID controller blocks
- *(Optional)* **Parallel Computing Toolbox** — for faster training

---

## Quick Start

```matlab
% 1. Open MATLAB and navigate to the project root
cd path/to/Quadcopter_control_system

% 2. Run setup — loads physical parameters and creates the SAC agent
run('setup.m')

% 3. Build the Simulink models programmatically
run('scripts/build_simulink_model.m')

% 4. (Optional) Open the model to inspect wiring
open_system('models/quadcopter_rl_env.slx')

% 5. Train the SAC agent (~30–90 min depending on hardware)
run('scripts/train_agent.m')

% 6. Evaluate performance and generate comparison figures
run('scripts/evaluate_and_plot.m')

% 7. (Optional) Run the 3D flight visualization demo
visualize_flight()
```

---

## Project Phases

### Phase 1 — Plant Modelling
- Newton-Euler **6-DOF rigid body dynamics** via Aerospace Blockset
- 250mm X-frame quadcopter: mass = 0.468 kg, arm length = 0.225 m
- Thrust/torque mixing matrix maps motor commands to body forces and torques

### Phase 2 — Baseline PID Design
- **Cascaded architecture**: outer position loop (0.8 Hz bandwidth) → inner attitude loop (8 Hz bandwidth)
- 6 independent PID controllers for X, Y, Z, Roll, Pitch, and Yaw
- Manually tuned via step-response analysis
- Validates stable hover at z = 1 m in calm conditions

### Phase 3 — Wind Disturbance Injection
- **Step wind**: 5 N lateral force at t = 5 s (crosswind gust)
- **Sinusoidal wind**: 3 N amplitude at 2 Hz (oscillatory turbulence)
- **Stochastic turbulence**: random force perturbations
- Demonstrates baseline PID degradation under disturbances

### Phase 4 — RL Augmentation
- SAC agent observes **15-dimensional** normalised state vector
- Outputs **6 gain deltas** (ΔKp, ΔKi, ΔKd for roll and pitch)
- Effective gains: `K_eff = K_base + α · ΔK` where α = 0.5
- Trained over **3000 episodes** with domain randomisation (wind timing, magnitude, initial positions)

---

## Technical Details

### Observation Space (15 dimensions, normalised to [-1, 1])

| Index | Signal | Normalisation |
|:---:|---|---|
| 1–3 | Position errors (x, y, z) | ÷ 3 m |
| 4–6 | Attitude errors (φ, θ, ψ) | ÷ 0.5 rad |
| 7–9 | Velocity errors (dx, dy, dz) | ÷ 5 m/s |
| 10–12 | Angular rate errors (dφ, dθ, dψ) | ÷ 3 rad/s |
| 13–15 | Current effective gains (Kp, Ki, Kd) | ÷ [10, 2, 5] |

### Action Space (6 dimensions, [-1, 1])

| Index | Parameter | Target Loop |
|:---:|---|---|
| 1–3 | ΔKp, ΔKi, ΔKd | Inner PID — Roll (φ) |
| 4–6 | ΔKp, ΔKi, ΔKd | Inner PID — Pitch (θ) |

### Reward Function

```
R = −(w_pos · ‖e_pos‖² + w_att · ‖e_att‖² + w_gain · ‖ΔK‖²) + bonus − crash_penalty
```

| Component | Weight | Purpose |
|---|:---:|---|
| Position cost | 1.0 | Primary tracking objective |
| Attitude cost | 0.5 | Prevent excessive tilting |
| Gain regularisation | 0.1 | Penalise large gain adjustments |
| Stability bonus | 0.3 | Reward sustained low-error hover |
| Crash penalty | 100.0 | Discourage unsafe states |

### Neural Network Architecture

**Actor (Gaussian Policy):**
```
obs(15) → FC(256) → ReLU → FC(256) → ReLU → FC(128) → ReLU → FC(6) → tanh
```

**Twin Critics (Q-Value):**
```
obs(15) → FC(128) ──┐
                     ├─ Add → ReLU → FC(256) → ReLU → FC(1)
act(6)  → FC(128) ──┘
```

### Training Configuration

| Parameter | Value |
|---|---|
| Max episodes | 3,000 |
| Steps per episode | 500 (10 s at 50 Hz) |
| Learning rate | 3 × 10⁻⁴ |
| Replay buffer | 10⁶ transitions |
| Mini-batch size | 256 |
| Discount factor (γ) | 0.99 |
| Target smoothing (τ) | 0.005 |
| Warm-start steps | 5,000 |
| Entropy target | −6 (auto-tuned) |

---

## Visualization

The project includes a **3D animated flight visualization** (`visualize_flight.m`) that demonstrates:

1. **Takeoff** — Smooth ascent from ground to 1 m hover altitude
2. **Stable Hover** — Position holding with minimal drift
3. **Wind Disturbance** — Lateral gust impact with visible displacement
4. **PID + RL Compensation** — Real-time gain adaptation and recovery
5. **Recovery** — Return to stable hover position

Run it with:
```matlab
visualize_flight()
```

---

## License

This project is released under the [MIT License](LICENSE).

---

**Built with MATLAB & Simulink · Soft Actor-Critic · Deep Reinforcement Learning**
