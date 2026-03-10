% init_params.m
% Populates the MATLAB workspace with all variables referenced by name
% in Simulink block parameters. Must be run before opening the .slx model.
% Variables defined here are used as block mask parameters in the Simulink
% model (e.g., 'm' in the 6DOF block, 'Kp_phi' in PID blocks, etc.).

fprintf('=== Initializing Parameters ===\n\n');

% 1. Physical Parameters (250mm X-frame quadcopter)
m   = 0.468;         % Total mass [kg]
g   = 9.81;          % Gravity [m/s^2]
Ixx = 4.856e-3;      % Roll inertia [kg*m^2]
Iyy = 4.856e-3;      % Pitch inertia [kg*m^2]
Izz = 8.801e-3;      % Yaw inertia [kg*m^2]
Jr  = 3.357e-5;      % Rotor spin inertia [kg*m^2]
l   = 0.225;         % Arm length centre-to-rotor [m]
k   = 2.980e-6;      % Thrust coefficient [N*s^2]
d   = 1.140e-7;      % Drag/torque coefficient [N*m*s^2]
Ax  = 0.01;          % Linear drag X [kg/s]
Ay  = 0.01;          % Linear drag Y [kg/s]
Az  = 0.02;          % Linear drag Z [kg/s]

% 2. Hover Equilibrium
F_hover     = m * g / 4;            % Per-rotor thrust at hover [N]
omega_hover = sqrt(F_hover / k);    % Per-rotor angular speed at hover [rad/s]
fprintf('Hover rotor speed: %.1f rad/s (%.0f RPM)\n', ...
    omega_hover, omega_hover * 60 / (2 * pi));

% 3. Baseline PID Gains
% Outer loop - Position (slow, ~0.8 Hz bandwidth)
Kp_x = 0.25;
Ki_x = 0.02;
Kd_x = 0.12;
Kp_y = 0.25;
Ki_y = 0.02;
Kd_y = 0.12;
Kp_z = 0.50;
Ki_z = 0.05;
Kd_z = 0.20;

% Inner loop - Attitude (fast, ~8 Hz bandwidth)
Kp_phi   = 4.50;
Ki_phi   = 0.50;
Kd_phi   = 1.80;
Kp_theta = 4.50;
Ki_theta = 0.50;
Kd_theta = 1.80;
Kp_psi   = 2.00;
Ki_psi   = 0.10;
Kd_psi   = 0.50;

% Baseline gain vectors for RL scaling
K_base_phi   = [Kp_phi;   Ki_phi;   Kd_phi];
K_base_theta = [Kp_theta; Ki_theta; Kd_theta];

% 4. RL Agent Parameters
alpha   = 0.5;       % Gain adjustment scale: K_eff = K_base + alpha*delta_K
Kp_max  = [9.0; 9.0; 4.0];    % Max Kp [phi; theta; psi]
Ki_max  = [1.5; 1.5; 0.5];    % Max Ki
Kd_max  = [4.0; 4.0; 1.5];    % Max Kd
Kp_min  = [0.5; 0.5; 0.2];    % Min Kp
Ki_min  = [0.0; 0.0; 0.0];    % Min Ki
Kd_min  = [0.1; 0.1; 0.05];   % Min Kd

% 5. Simulation Parameters
Ts_inner    = 0.002;    % Plant/inner loop sample time [s] (500 Hz)
Ts_outer    = 0.02;     % Outer loop/RL agent sample time [s] (50 Hz)
T_episode   = 10;       % Episode duration [s]

% Hover setpoint
x_d   = 0;       % Desired X position [m]
y_d   = 0;       % Desired Y position [m]
z_d   = 1.0;     % Desired Z position [m] (1 m hover altitude)
psi_d = 0;       % Desired yaw [rad]

% 6. Wind Disturbance
wind_onset_time = 5.0;    % Time at which wind step occurs [s]
wind_force_N    = 5.0;    % Wind force magnitude [N]

% 7. Normalisation Factors (for observation vector)
norm_pos   = 3.0;     % Position error normalisation [m]
norm_att   = 0.5;     % Attitude error normalisation [rad]
norm_vel   = 5.0;     % Velocity error normalisation [m/s]
norm_rate  = 3.0;     % Angular rate normalisation [rad/s]
norm_Kp    = 10.0;    % Kp normalisation
norm_Ki    = 2.0;     % Ki normalisation
norm_Kd    = 5.0;     % Kd normalisation

fprintf('\nParameters loaded. Run create_sac_agent.m next.\n');
fprintf('=== init_params.m complete ===\n');
