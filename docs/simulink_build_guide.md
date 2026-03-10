# Simulink Build Guide — quadcopter_rl_env.slx

> **Note:** The Simulink model can be built automatically by running
> `scripts/build_simulink_model.m`. This document describes the wiring
> for manual reference and debugging.

## Solver Settings
- Type: Fixed-step | Solver: ode4 | Step size: 0.002s | Stop time: 10s

## Block List and Library Sources

| Block Name | Library Path | Key Settings |
|---|---|---|
| 6DOF (Euler Angles) | Aerospace Blockset → Equations of Motion | Mass=m, Inertia=diag([Ixx Iyy Izz]) |
| PID Controller (×6) | Simulink → Continuous → PID Controller | Parallel form, Continuous, External gain inputs ON for inner loop PIDs |
| RL Agent Block | Reinforcement Learning Toolbox → RL Agent | Agent=sac_agent, Ts=0.02 |
| MATLAB Function (×3) | Simulink → User-Defined Functions | Thrust_Mixer, Reward_Calculator, Crash_Detector |
| Rate Transition | Simulink → Signal Attributes | Output Ts=0.02 (outer loop) |
| Step | Simulink → Sources | Time=wind_onset_time, After=wind_force_N |
| Mux (15-input) | Simulink → Signal Routing | Builds observation vector |
| Demux (6-output) | Simulink → Signal Routing | Splits RL action vector |
| Sum | Simulink → Math Operations | ++ for wind injection, +- for error signals |
| Saturation | Simulink → Discontinuities | Angle limits: ±0.3 rad |
| Transfer Function | Simulink → Continuous | N/(s+N) filtered derivative, N=100 |

## Wiring Order

### Step 1 — Plant Subsystem
1. Place `6DOF (Euler Angles)` block
2. Add `Thrust_Mixer` MATLAB Function block upstream
3. Add `Sum (++)` block between Thrust_Mixer force output and 6DOF force input — **this is the wind injection point**
4. Route 6DOF state outputs through Bus Selector to extract all 12 state elements
5. Wrap in a Subsystem: `Plant_6DOF`

### Step 2 — Inner Attitude Loop (500 Hz)
1. Place `PID_Roll`, `PID_Pitch`, `PID_Yaw` blocks
2. Enable external gain ports on `PID_Roll` and `PID_Pitch` (double-click → change P/I/D from Internal to External)
3. For each external gain port, insert `Add (++)` block:
   - Input 1: Constant block (`Kp_phi`, `Ki_phi`, etc.)
   - Input 2: Scaled RL action (connected in Step 4)
4. Wire attitude errors (desired − actual) to PID inputs
5. Connect PID outputs to U2, U3, U4 inputs of Plant_6DOF

### Step 3 — Outer Position Loop (50 Hz)
1. Place `PID_X`, `PID_Y`, `PID_Z` blocks
2. No external gain ports needed (outer loop not adapted by RL)
3. Add Constant setpoint blocks: `x_d=0`, `y_d=0`, `z_d=1`
4. Wire position errors to PID inputs
5. `PID_X` output → Saturation (±0.3 rad) → `theta_desired`
6. `PID_Y` output → Saturation (±0.3 rad) → `phi_desired`
7. `PID_Z` output → `U1` of Plant_6DOF
8. Insert `Rate Transition` block (Ts=0.02) between outer and inner loops

### Step 4 — RL Agent Block
1. Place `RL Agent Block` from RL Toolbox library
2. Set Agent = `sac_agent` (workspace variable), Ts = 0.02
3. Build 15-element observation Mux:
   - Positions 1–6: error signals from Sum blocks
   - Positions 7–12: filtered derivatives via Transfer Function N/(s+N), N=100
   - Positions 13–15: tap effective Kp/Ki/Kd signals after the Add blocks
4. Route Mux output through `Normalize_Obs` MATLAB Function block → RL Agent obs port
5. Connect RL Agent action output → Demux (6 outputs) → Gain (alpha=0.5) per output → Add blocks on PID_Roll and PID_Pitch gain ports
6. Connect `Reward_Calculator` output → RL Agent reward port

### Step 5 — Wind Disturbance
1. Place `Step` block: Time=`wind_onset_time`, After=`wind_force_N`
2. Place two `Constant` blocks: value=0 (for Fy and Fz)
3. Bundle through `Mux (3→1)` → connect to the wind Sum block **inside Plant_6DOF**

### Step 6 — Reward and Crash Detection
1. Place `Reward_Calculator` MATLAB Function block
   - Inputs: `e_pos` (3×1), `e_att` (3×1), `delta_K` (6×1), `crashed` (scalar)
   - Paste the `compute_reward` function body from `reward_function.m`
2. Place `Crash_Detector` MATLAB Function block:
```matlab
function crashed = detect_crash(x, y, z, phi, theta)
    crashed = double(sqrt(x^2+y^2+(z-1)^2) > 3.0 || sqrt(phi^2+theta^2) > 1.2);
end
```
3. Connect `Crash_Detector` output to `Stop Simulation` block and to `Reward_Calculator`

## Normalisation Reference
| Signal | Physical range | Scale factor |
|---|---|---|
| Position errors | ±3 m | 3 |
| Attitude errors | ±0.5 rad | 0.5 |
| Velocity errors | ±5 m/s | 5 |
| Angular rate errors | ±3 rad/s | 3 |
| Effective Kp | 0–10 | 10 |
| Effective Ki | 0–2 | 2 |
| Effective Kd | 0–5 | 5 |

## Execution Order Reminder
```matlab
run('setup.m')                         % must be first
run('scripts/build_simulink_model.m')  % builds .slx models
open_system('models/quadcopter_rl_env.slx')
run('scripts/train_agent.m')           % 30–90 min
run('scripts/evaluate_and_plot.m')
```
