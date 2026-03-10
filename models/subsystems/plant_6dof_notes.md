# Plant 6-DOF Subsystem — Block Configuration Reference

## Overview
The `Plant_6DOF` subsystem encapsulates the quadcopter rigid-body dynamics
using the Aerospace Blockset `6DOF (Euler Angles)` block and a custom
`Thrust_Mixer` MATLAB Function block.

## 6DOF Block Configuration
| Parameter | Value | Source |
|-----------|-------|--------|
| Mass | `m` (0.468 kg) | init_params.m |
| Inertia matrix | `diag([Ixx, Iyy, Izz])` | init_params.m |
| Initial position | `[0; 0; 1]` (hover at 1 m) | Set in block |
| Initial velocity | `[0; 0; 0]` | Set in block |
| Initial Euler angles | `[0; 0; 0]` | Set in block |
| Initial body rates | `[0; 0; 0]` | Set in block |

## Thrust Mixer
Maps 4 control inputs `[U1, U2, U3, U4]` to body-frame forces and moments:
- `U1` = Total thrust (from altitude PID)
- `U2` = Roll torque (from roll PID)
- `U3` = Pitch torque (from pitch PID)
- `U4` = Yaw torque (from yaw PID)

```matlab
function [Fb, Mb] = thrust_mixer(U1, U2, U3, U4)
    Fb = [0; 0; -U1];    % Body-frame force (NED convention: thrust is -Z)
    Mb = [U2; U3; U4];   % Body-frame moments
end
```

## Outputs (12 state variables)
| Index | Variable | Unit |
|-------|----------|------|
| 1–3 | x, y, z (NED position) | m |
| 4–6 | u, v, w (body velocities) | m/s |
| 7–9 | φ, θ, ψ (Euler angles) | rad |
| 10–12 | p, q, r (body rates) | rad/s |

## Wind Injection Point
External forces are added to `Fb` at the `Sum (++)` block between the
Thrust_Mixer output and the 6DOF block force input. Wind force is applied
in the NED X-axis direction only.
