# Architecture Overview — Quadcopter Autonomous Control

## 1. Problem Statement

A classically tuned cascaded PID controller achieves precise position tracking
in calm conditions but suffers from significant performance degradation under
external wind disturbances. Fixed PID gains represent a compromise between
responsiveness and stability, and cannot adapt to changing environmental
conditions in real-time.

## 2. Proposed Solution

A **Soft Actor-Critic (SAC)** reinforcement learning agent acts as an
intelligent supervisory layer that dynamically adjusts the inner PID loop's
gain parameters (Kp, Ki, Kd) in real-time. The agent **does not replace**
the PID controller — it tunes it continuously based on observed tracking
errors and current system state.

### Why SAC?
- **Off-policy**: sample-efficient, reuses past experience via replay buffer
- **Entropy regularisation**: naturally explores the gain space, avoids
  premature convergence to suboptimal gain schedules
- **Continuous actions**: directly outputs smooth gain adjustments without
  discretisation artefacts
- **Stable training**: twin critic architecture prevents Q-value overestimation

## 3. System Architecture

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

## 4. Four Project Phases

### Phase 1 — Plant Modelling
- Newton-Euler 6-DOF rigid body dynamics
- 250mm X-frame quadcopter (m=0.468 kg, l=0.225 m)
- Aerospace Blockset `6DOF (Euler Angles)` block
- Thrust/torque mixing matrix maps motor commands to body forces

### Phase 2 — Baseline PID Design
- Cascaded architecture: outer position loop (0.8 Hz) → inner attitude loop (8 Hz)
- 6 independent PID controllers (X, Y, Z, Roll, Pitch, Yaw)
- Manual tuning via step response analysis
- Validates stable hover at z=1 m in calm conditions

### Phase 3 — Wind Disturbance Injection
- Step force disturbance: 5 N at t=5 s (simulates crosswind gust)
- Demonstrates baseline PID degradation (0.24 m RMSE vs 0.002 m calm)
- Additional scenarios: sinusoidal wind, stochastic turbulence

### Phase 4 — RL Augmentation
- SAC agent observes 15-dimensional state (errors + rates + current gains)
- Outputs 6 gain deltas for roll and pitch inner loops
- Effective gains: `K_eff = K_base + alpha * delta_K` (alpha=0.5)
- Trained over 3000 episodes with domain randomisation
- Achieves 65–71% RMSE improvement under wind disturbances

## 5. Observation and Action Spaces

### Observation (15 dimensions, normalised to [-1, 1])
| Index | Signal | Normalisation |
|-------|--------|---------------|
| 1–3 | Position errors (x, y, z) | ÷ 3 m |
| 4–6 | Attitude errors (φ, θ, ψ) | ÷ 0.5 rad |
| 7–9 | Velocity errors (dx, dy, dz) | ÷ 5 m/s |
| 10–12 | Angular rate errors (dφ, dθ, dψ) | ÷ 3 rad/s |
| 13–15 | Current effective gains (Kp, Ki, Kd) | ÷ [10, 2, 5] |

### Action (6 dimensions, [-1, 1])
| Index | Parameter | Target Loop |
|-------|-----------|-------------|
| 1–3 | ΔKp, ΔKi, ΔKd for Roll | Inner PID (φ) |
| 4–6 | ΔKp, ΔKi, ΔKd for Pitch | Inner PID (θ) |

## 6. Reward Function

```
R = -(w_pos·‖e_pos‖² + w_att·‖e_att‖² + w_gain·‖ΔK‖²) + bonus - crash_penalty
```

| Component | Weight | Purpose |
|-----------|--------|---------|
| Position cost | 1.0 | Primary tracking objective |
| Attitude cost | 0.5 | Prevent excessive tilting |
| Gain cost | 0.1 | Regularise gain adjustments |
| Stability bonus | 0.3 | Sustained low-error hover reward |
| Crash penalty | 100.0 | Discourage unsafe states |

## 7. Network Architecture

### Actor (Gaussian Policy)
```
obs(15) → FC(256) → ReLU → FC(256) → ReLU → FC(128) → ReLU → FC(6) → tanh
```

### Twin Critics (Q-Value)
```
obs(15) → FC(128) ──┐
                     ├─ Add → ReLU → FC(256) → ReLU → FC(1)
act(6)  → FC(128) ──┘
```

## 8. Training Configuration

| Parameter | Value |
|-----------|-------|
| Max episodes | 3000 |
| Steps per episode | 500 (10s at 50 Hz) |
| Learning rate | 3×10⁻⁴ (actor and critics) |
| Replay buffer | 10⁶ transitions |
| Mini-batch | 256 |
| Discount factor | 0.99 |
| Target smoothing | 0.005 |
| Warm-start steps | 5000 |
| Entropy | Auto-tuned (target = -6) |
