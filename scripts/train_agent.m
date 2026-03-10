% train_agent.m
% Wraps the Simulink model as an RL environment and calls train().
% Run after setup.m (which runs init_params.m and create_sac_agent.m).
%
% Prerequisites:
%   - setup.m has been run (workspace populated)
%   - build_simulink_model.m has been run (quadcopter_rl_env.slx exists)

fprintf('=== Starting SAC Training ===\n\n');

% 0. Verify prerequisites
required = {'m', 'Ixx', 'Kp_phi', 'sac_agent', 'obsInfo', 'actInfo', ...
            'Ts_outer', 'T_episode'};
for i = 1:numel(required)
    assert(exist(required{i}, 'var') == 1, ...
        'Missing variable: %s - run setup.m first.', required{i});
end

% 1. Create Simulink RL Environment
model_name  = 'quadcopter_rl_env';
agent_block = [model_name '/RL_Agent'];

% Load the model into memory (does not open it visually)
load_system(fullfile(pwd, 'models', [model_name '.slx']));

env = rlSimulinkEnv(model_name, agent_block, obsInfo, actInfo);

% 2. Episode Reset Function - randomises disturbance each episode
env.ResetFcn = @(in) localResetFcn(in, model_name);

% 3. Training Options
maxSteps = floor(T_episode / Ts_outer);   % 10s / 0.02s = 500 steps

train_opts = rlTrainingOptions( ...
    'MaxEpisodes',                3000, ...
    'MaxStepsPerEpisode',         maxSteps, ...
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria',       'AverageReward', ...
    'StopTrainingValue',          -15, ...
    'SaveAgentCriteria',          'EpisodeReward', ...
    'SaveAgentValue',             -20, ...
    'SaveAgentDirectory',         fullfile(pwd, 'saved_agents'), ...
    'Verbose',                    true, ...
    'Plots',                      'training-progress');

% 4. Train
fprintf('Training started - estimated 30-90 min depending on hardware.\n');
fprintf('  Max episodes:  %d\n', train_opts.MaxEpisodes);
fprintf('  Steps/episode: %d\n', maxSteps);
fprintf('  Stop criterion: avg reward >= %.0f over %d episodes\n\n', ...
    train_opts.StopTrainingValue, train_opts.ScoreAveragingWindowLength);

tic;
training_stats = train(sac_agent, env, train_opts);
elapsed = toc;
fprintf('\nTraining complete in %.1f min (%d episodes).\n', ...
    elapsed / 60, numel(training_stats.EpisodeReward));

% 5. Save trained agent
out_path = fullfile(pwd, 'saved_agents', 'sac_trained_final.mat');
save(out_path, 'sac_agent', 'training_stats', 'obsInfo', 'actInfo');
fprintf('Agent saved to: %s\n', out_path);

fprintf('\nRun scripts/evaluate_and_plot.m to generate results.\n');
fprintf('=== train_agent.m complete ===\n');


% ======================================================================
%  LOCAL FUNCTIONS
% ======================================================================

function in = localResetFcn(in, model_name)
    % LOCALRESETFCN  Randomise episode initial conditions.
    %   - Wind onset time:   uniform [3, 7] s
    %   - Wind magnitude:    uniform [2, 7] N
    %   - Initial X setpoint: small Gaussian perturbation
    %   - Initial Y setpoint: small Gaussian perturbation

    % Randomise wind step parameters
    in = setBlockParameter(in, ...
        [model_name '/Wind_Step_Fx'], 'Time', num2str(3 + 4*rand()));
    in = setBlockParameter(in, ...
        [model_name '/Wind_Step_Fx'], 'After', num2str(2 + 5*rand()));

    % Small initial position perturbation
    in = setBlockParameter(in, ...
        [model_name '/Setpoint_X'], 'Value', num2str(0.1*randn()));
    in = setBlockParameter(in, ...
        [model_name '/Setpoint_Y'], 'Value', num2str(0.1*randn()));
end
