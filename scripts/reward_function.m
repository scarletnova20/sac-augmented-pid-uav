% reward_function.m
% Canonical reward logic for the quadcopter RL agent.
%
% This exact function body must be copy-pasted into the MATLAB Function
% block named "Reward_Calculator" inside the Simulink model.
%
% Also includes unit tests - run via:  reward_function()

function reward_function()
    % Entry point: run unit tests when called without arguments
    run_tests();
end

% ======================================================================
%  CORE REWARD FUNCTION
% ======================================================================

function reward = compute_reward(e_pos, e_att, delta_K, crashed)
    % COMPUTE_REWARD  Calculate step reward for the RL agent.
    %
    %   Inputs:
    %     e_pos    - 3x1 position error vector [ex; ey; ez] (m)
    %     e_att    - 3x1 attitude error vector [ephi; etheta; epsi] (rad)
    %     delta_K  - 6x1 gain adjustment vector (from RL action)
    %     crashed  - scalar flag (>0.5 means crash detected)
    %
    %   Output:
    %     reward   - scalar reward clipped to [-110, 10]

    % Weights
    w_pos    = 1.0;       % Position tracking priority
    w_att    = 0.5;       % Attitude tracking priority
    w_gain   = 0.1;       % Penalty for large gain adjustments (regulariser)
    w_stable = 0.3;       % Bonus for sustained low-error hover
    CRASH_PENALTY = 100.0;

    % Quadratic costs
    pos_cost  = w_pos  * (e_pos'  * e_pos);
    att_cost  = w_att  * (e_att'  * e_att);
    gain_cost = w_gain * (delta_K' * delta_K);

    % Stability bonus for sustained low-error hover
    if pos_cost < 0.05 && att_cost < 0.02
        bonus = w_stable;
    else
        bonus = 0;
    end

    % Crash penalty
    crash_cost = double(crashed > 0.5) * CRASH_PENALTY;

    % Compute and clip
    reward = -(pos_cost + att_cost + gain_cost) + bonus - crash_cost;
    reward = max(min(reward, 10.0), -110.0);
end

% ======================================================================
%  CRASH DETECTION FUNCTION
% ======================================================================

function crashed = detect_crash(x, y, z, phi, theta)
    % DETECT_CRASH  Check if the quadcopter has left the safe envelope.
    %
    %   Crash conditions:
    %     - Position deviation > 3.0 m from hover point (0,0,1)
    %     - Roll/pitch angle magnitude > 1.2 rad (~69 deg)

    pos_dev = sqrt(x^2 + y^2 + (z - 1)^2);
    att_dev = sqrt(phi^2 + theta^2);
    crashed = double(pos_dev > 3.0 || att_dev > 1.2);
end

% ======================================================================
%  UNIT TESTS
% ======================================================================

function run_tests()
    fprintf('\n--- Reward Unit Tests ---\n');

    % Test 1: Perfect hover - should receive stability bonus
    r1 = compute_reward([0;0;0], [0;0;0], zeros(6,1), 0);
    fprintf('Perfect hover:    r = %+.4f  (expect ~+0.30)\n', r1);
    assert(abs(r1 - 0.30) < 0.01, 'Test 1 FAILED');

    % Test 2: Moderate disturbance - position and attitude error
    r2 = compute_reward([0.5;0;0], [0.1;0;0], zeros(6,1), 0);
    fprintf('5N wind 0.5m err: r = %+.4f  (expect ~-0.26)\n', r2);
    assert(r2 < 0 && r2 > -1.0, 'Test 2 FAILED');

    % Test 3: Large gain delta - regularisation penalty kicks in
    r3 = compute_reward([0.1;0;0], [0.05;0;0], ones(6,1), 0);
    fprintf('Large gain delta: r = %+.4f  (expect ~-0.61)\n', r3);
    assert(r3 < 0, 'Test 3 FAILED');

    % Test 4: Crash event - heavy penalty
    r4 = compute_reward([2;0;0], [0.8;0;0], zeros(6,1), 1);
    fprintf('Crash event:      r = %+.4f  (expect ~-104)\n', r4);
    assert(r4 < -100, 'Test 4 FAILED');

    % Test 5: Crash detection - within safe envelope
    c1 = detect_crash(0, 0, 1, 0, 0);
    assert(c1 == 0, 'Test 5 FAILED: safe position flagged as crash');

    % Test 6: Crash detection - position overflow
    c2 = detect_crash(4, 0, 1, 0, 0);
    assert(c2 == 1, 'Test 6 FAILED: position overflow not detected');

    % Test 7: Crash detection - attitude overflow
    c3 = detect_crash(0, 0, 1, 1.5, 0);
    assert(c3 == 1, 'Test 7 FAILED: attitude overflow not detected');

    fprintf('All reward unit tests PASSED.\n\n');
end
