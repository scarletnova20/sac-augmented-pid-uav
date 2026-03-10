% setup.m - Run this first to initialise the entire project
% Master setup script that adds all project folders to the MATLAB path,
% loads physical/simulation parameters, and creates the SAC agent.
%
% Usage:
%   cd path/to/quadcopter_rl_project
%   run('setup.m')

clc; clear; close all;
fprintf('=== Quadcopter RL Project Setup ===\n\n');

% Add all project subdirectories to the MATLAB path
addpath(genpath(pwd));

% Step 1: Load physical constants, PID gains, and simulation parameters
run('scripts/init_params.m');

% Step 2: Build SAC agent with actor/critic networks
run('scripts/create_sac_agent.m');

fprintf('\n=== Setup Complete ===\n');
fprintf('Next steps:\n');
fprintf('  1. Run build_simulink_model() to generate .slx files\n');
fprintf('  2. Open models/quadcopter_rl_env.slx to inspect the model\n');
fprintf('  3. Run scripts/train_agent.m to start RL training\n');
