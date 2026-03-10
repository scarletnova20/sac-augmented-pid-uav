% create_sac_agent.m
% Defines observation/action spaces, builds Actor + Twin Critic networks,
% and instantiates the SAC agent. Requires init_params.m to have run first.
%
% Compatible with MATLAB R2024b rlContinuousGaussianActor API.

fprintf('=== Creating SAC Agent ===\n\n');

% 1. Observation Space (15 elements, normalised to [-1, 1])
obs_dim = 15;
obsInfo = rlNumericSpec([obs_dim, 1], ...
    'LowerLimit', -ones(obs_dim, 1), ...
    'UpperLimit',  ones(obs_dim, 1));
obsInfo.Name        = 'QuadcopterObservation';
obsInfo.Description = 'Errors, derivatives, and current effective gains (normalised)';

% 2. Action Space (6 elements - inner loop gain deltas)
act_dim = 6;
actInfo = rlNumericSpec([act_dim, 1], ...
    'LowerLimit', -ones(act_dim, 1), ...
    'UpperLimit',  ones(act_dim, 1));
actInfo.Name        = 'GainAdjustments';
actInfo.Description = 'Normalised PID gain deltas for roll and pitch inner loops';

% 3. Actor Network - stochastic Gaussian policy (R2024b dual-output)
% Two output branches from shared hidden layers:
%   mean_out  : FC -> tanh       (bounded action mean)
%   std_out   : FC -> softplus   (positive std deviation)

commonPath = [
    featureInputLayer(obs_dim, 'Normalization', 'none', 'Name', 'obs_in')
    fullyConnectedLayer(256, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(256, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(128, 'Name', 'fc3')
    reluLayer('Name', 'relu3')
];

meanPath = [
    fullyConnectedLayer(act_dim, 'Name', 'fc_mean')
    tanhLayer('Name', 'mean_out')
];

stdPath = [
    fullyConnectedLayer(act_dim, 'Name', 'fc_std')
    softplusLayer('Name', 'std_out')
];

actorLG = layerGraph(commonPath);
actorLG = addLayers(actorLG, meanPath);
actorLG = addLayers(actorLG, stdPath);
actorLG = connectLayers(actorLG, 'relu3', 'fc_mean');
actorLG = connectLayers(actorLG, 'relu3', 'fc_std');
actorNet = dlnetwork(actorLG);

actor = rlContinuousGaussianActor(actorNet, obsInfo, actInfo, ...
    'ActionMeanOutputNames', 'mean_out', ...
    'ActionStandardDeviationOutputNames', 'std_out', ...
    'ObservationInputNames', 'obs_in');

% 4. Twin Critic Networks
criticNet1 = build_critic_network(obs_dim, act_dim, 'c1');
criticNet2 = build_critic_network(obs_dim, act_dim, 'c2');

critic1 = rlQValueFunction(criticNet1, obsInfo, actInfo, ...
    'ObservationInputNames', 'oi_c1', ...
    'ActionInputNames', 'ai_c1');
critic2 = rlQValueFunction(criticNet2, obsInfo, actInfo, ...
    'ObservationInputNames', 'oi_c2', ...
    'ActionInputNames', 'ai_c2');

% 5. SAC Agent Options (R2024b compatible)
opts = rlSACAgentOptions;
opts.ActorOptimizerOptions.LearnRate     = 3e-4;
opts.CriticOptimizerOptions(1).LearnRate = 3e-4;
opts.CriticOptimizerOptions(2).LearnRate = 3e-4;
opts.ExperienceBufferLength              = 1e6;
opts.MiniBatchSize                       = 256;
opts.DiscountFactor                      = 0.99;
opts.TargetSmoothFactor                  = 0.005;
opts.EntropyWeightOptions.EntropyWeight  = 1;
opts.EntropyWeightOptions.TargetEntropy  = -act_dim;
opts.EntropyWeightOptions.LearnRate      = 3e-4;
opts.NumWarmStartSteps                   = 5000;
opts.PolicyUpdateFrequency               = 1;
opts.SampleTime                          = Ts_outer;

% 6. Instantiate Agent
sac_agent = rlSACAgent(actor, [critic1, critic2], opts);

fprintf('SAC agent created successfully.\n');
fprintf('  Obs dim: %d | Act dim: %d\n', obs_dim, act_dim);
fprintf('  Replay buffer: %.0e | Batch: %d\n', ...
    opts.ExperienceBufferLength, opts.MiniBatchSize);
fprintf('=== create_sac_agent.m complete ===\n');


% ======================================================================
%  LOCAL FUNCTIONS
% ======================================================================

function net = build_critic_network(obs_d, act_d, sfx)
    obs_path = [
        featureInputLayer(obs_d, 'Normalization', 'none', 'Name', ['oi_' sfx])
        fullyConnectedLayer(128, 'Name', ['ofc_' sfx])
    ];
    act_path = [
        featureInputLayer(act_d, 'Normalization', 'none', 'Name', ['ai_' sfx])
        fullyConnectedLayer(128, 'Name', ['afc_' sfx])
    ];
    common = [
        additionLayer(2, 'Name', ['add_' sfx])
        reluLayer('Name', ['rl_' sfx])
        fullyConnectedLayer(256, 'Name', ['fc2_' sfx])
        reluLayer('Name', ['rl2_' sfx])
        fullyConnectedLayer(1, 'Name', ['qout_' sfx])
    ];
    lg = layerGraph(obs_path);
    lg = addLayers(lg, act_path);
    lg = addLayers(lg, common);
    lg = connectLayers(lg, ['ofc_' sfx], ['add_' sfx '/in1']);
    lg = connectLayers(lg, ['afc_' sfx], ['add_' sfx '/in2']);
    net = dlnetwork(lg);
end
