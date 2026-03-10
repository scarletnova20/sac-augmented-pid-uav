function build_simulink_model()
m=evalin('base','m'); g=evalin('base','g');
Ixx=evalin('base','Ixx'); Iyy=evalin('base','Iyy'); Izz=evalin('base','Izz');
Ax=evalin('base','Ax'); Ay=evalin('base','Ay'); Az=evalin('base','Az');
Ts_inner=evalin('base','Ts_inner');
norm_pos=evalin('base','norm_pos'); norm_att=evalin('base','norm_att');
norm_vel=evalin('base','norm_vel'); norm_rate=evalin('base','norm_rate');
norm_Kp=evalin('base','norm_Kp'); norm_Ki=evalin('base','norm_Ki'); norm_Kd=evalin('base','norm_Kd');

fprintf('=== Building Simulink Models ===\n\n');
models_dir = fullfile(pwd, 'models');
if ~isfolder(models_dir); mkdir(models_dir); end

sixdof_code = build_sixdof_code(m, g, Ixx, Iyy, Izz, Ax, Ay, Az, Ts_inner);
mixer_code = build_mixer_code();
normalize_code = build_normalize_code(norm_pos, norm_att, norm_vel, norm_rate, norm_Kp, norm_Ki, norm_Kd);
reward_code = build_reward_code();
crash_code = build_crash_code();

mdl = 'quadcopter_rl_env';
fprintf('Building %s.slx ...\n', mdl);
if bdIsLoaded(mdl); close_system(mdl, 0); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', 'Ts_inner', 'StopTime', 'T_episode', 'SaveOutput', 'on', 'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on');
fprintf('  Phase A: Adding blocks...\n');
add_block('simulink/Sources/Constant', [mdl '/Setpoint_X'], 'Value', 'x_d', 'Position', [50 50 100 70]);
add_block('simulink/Sources/Constant', [mdl '/Setpoint_Y'], 'Value', 'y_d', 'Position', [50 120 100 140]);
add_block('simulink/Sources/Constant', [mdl '/Setpoint_Z'], 'Value', 'z_d', 'Position', [50 190 100 210]);
add_block('simulink/Sources/Constant', [mdl '/Setpoint_Psi'], 'Value', 'psi_d', 'Position', [50 260 100 280]);
plant = [mdl '/Plant_6DOF'];
add_block('simulink/Ports & Subsystems/Subsystem', plant, 'Position', [700 150 850 350]);
delete_line(plant, 'In1/1', 'Out1/1');
delete_block([plant '/In1']); delete_block([plant '/Out1']);
add_block('simulink/Sources/In1', [plant '/Forces_In'], 'Position', [30 80 60 100], 'Port', '1');
add_block('simulink/Sources/In1', [plant '/Moments_In'], 'Position', [30 160 60 180], 'Port', '2');
add_block('simulink/Sources/In1', [plant '/Wind_In'], 'Position', [30 240 60 260], 'Port', '3');
add_block('simulink/Math Operations/Add', [plant '/Wind_Sum'], 'Inputs', '++', 'Position', [150 80 180 110]);
add_block('simulink/User-Defined Functions/MATLAB Function', [plant '/SixDOF_Dynamics'], 'Position', [250 50 450 300]);
add_block('simulink/Sinks/Out1', [plant '/Pos_Out'], 'Position', [550 60 580 80], 'Port', '1');
add_block('simulink/Sinks/Out1', [plant '/Vel_Out'], 'Position', [550 120 580 140], 'Port', '2');
add_block('simulink/Sinks/Out1', [plant '/Euler_Out'], 'Position', [550 180 580 200], 'Port', '3');
add_block('simulink/Sinks/Out1', [plant '/Rates_Out'], 'Position', [550 240 580 260], 'Port', '4');
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Thrust_Mixer'], 'Position', [500 180 620 280]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Normalize_Obs'], 'Position', [950 130 1060 220]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Reward_Calculator'], 'Position', [900 400 1040 470]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Crash_Detector'], 'Position', [900 490 1040 540]);
add_block('simulink/Math Operations/Sum', [mdl '/Err_X'], 'Inputs', '+-', 'Position', [200 50 230 70]);
add_block('simulink/Math Operations/Sum', [mdl '/Err_Y'], 'Inputs', '+-', 'Position', [200 120 230 140]);
add_block('simulink/Math Operations/Sum', [mdl '/Err_Z'], 'Inputs', '+-', 'Position', [200 190 230 210]);
add_block('simulink/Math Operations/Sum', [mdl '/Err_Phi'], 'Inputs', '+-', 'Position', [200 350 230 370]);
add_block('simulink/Math Operations/Sum', [mdl '/Err_Theta'], 'Inputs', '+-', 'Position', [200 420 230 440]);
add_block('simulink/Math Operations/Sum', [mdl '/Err_Psi'], 'Inputs', '+-', 'Position', [200 490 230 510]);
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_Outer'], 'Position', [270 50 310 210], 'OutPortSampleTime', 'Ts_outer');
add_block('simulink/Continuous/PID Controller', [mdl '/PID_X'], 'Position', [350 40 430 80], 'P', 'Kp_x', 'I', 'Ki_x', 'D', 'Kd_x', 'Controller', 'PID');
add_block('simulink/Continuous/PID Controller', [mdl '/PID_Y'], 'Position', [350 110 430 150], 'P', 'Kp_y', 'I', 'Ki_y', 'D', 'Kd_y', 'Controller', 'PID');
add_block('simulink/Continuous/PID Controller', [mdl '/PID_Z'], 'Position', [350 180 430 220], 'P', 'Kp_z', 'I', 'Ki_z', 'D', 'Kd_z', 'Controller', 'PID');
add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_Theta'], 'UpperLimit', '0.3', 'LowerLimit', '-0.3', 'Position', [460 40 500 80]);
add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_Phi'], 'UpperLimit', '0.3', 'LowerLimit', '-0.3', 'Position', [460 110 500 150]);
add_block('simulink/Continuous/PID Controller', [mdl '/PID_Roll'], 'Position', [380 340 460 380], 'P', 'Kp_phi', 'I', 'Ki_phi', 'D', 'Kd_phi', 'Controller', 'PID');
add_block('simulink/Continuous/PID Controller', [mdl '/PID_Pitch'], 'Position', [380 410 460 450], 'P', 'Kp_theta', 'I', 'Ki_theta', 'D', 'Kd_theta', 'Controller', 'PID');
add_block('simulink/Continuous/PID Controller', [mdl '/PID_Yaw'], 'Position', [380 480 460 520], 'P', 'Kp_psi', 'I', 'Ki_psi', 'D', 'Kd_psi', 'Controller', 'PID');
for ch = 1:6
    add_block('simulink/Math Operations/Gain', sprintf([mdl '/Alpha_Scale_%d'], ch), 'Gain', 'alpha', 'Position', [800, 350+ch*40, 840, 370+ch*40]);
end
add_block('simulink/Sources/Step', [mdl '/Wind_Step_Fx'], 'Time', 'wind_onset_time', 'Before', '0', 'After', 'wind_force_N', 'Position', [500 500 550 520]);
add_block('simulink/Sources/Constant', [mdl '/Wind_Fy'], 'Value', '0', 'Position', [500 540 550 560]);
add_block('simulink/Sources/Constant', [mdl '/Wind_Fz'], 'Value', '0', 'Position', [500 580 550 600]);
add_block('simulink/Signal Routing/Mux', [mdl '/Wind_Mux'], 'Inputs', '3', 'Position', [590 500 610 600]);
add_block('simulink/Signal Routing/Mux', [mdl '/Obs_Mux'], 'Inputs', '15', 'Position', [900 50 920 350]);
for i = 1:6
    add_block('simulink/Continuous/Transfer Fcn', sprintf([mdl '/DerivFilter_%d'], i), 'Numerator', '[100 0]', 'Denominator', '[1 100]', 'Position', [750, 50+i*45, 810, 70+i*45]);
end
add_block('rllib/RL Agent', [mdl '/RL_Agent'], 'Position', [1100 130 1220 220]);
add_block('simulink/Signal Routing/Demux', [mdl '/Action_Demux'], 'Outputs', '6', 'Position', [1260 130 1280 280]);
add_block('simulink/Sinks/Stop Simulation', [mdl '/Stop_Sim'], 'Position', [1080 500 1120 530]);
add_block('simulink/Signal Routing/Demux', [mdl '/Pos_Demux'], 'Outputs', '3', 'Position', [900 150 920 210]);
add_block('simulink/Signal Routing/Demux', [mdl '/Euler_Demux'], 'Outputs', '3', 'Position', [900 250 920 310]);
add_block('simulink/Sources/Constant', [mdl '/Tap_Kp'], 'Value', 'Kp_phi', 'Position', [800 50 840 70]);
add_block('simulink/Sources/Constant', [mdl '/Tap_Ki'], 'Value', 'Ki_phi', 'Position', [800 80 840 100]);
add_block('simulink/Sources/Constant', [mdl '/Tap_Kd'], 'Value', 'Kd_phi', 'Position', [800 110 840 130]);
add_block('simulink/Signal Routing/Mux', [mdl '/Err_Pos_Mux'], 'Inputs', '3', 'Position', [800 400 820 440]);
add_block('simulink/Signal Routing/Mux', [mdl '/Err_Att_Mux'], 'Inputs', '3', 'Position', [800 450 820 490]);
add_block('simulink/Signal Routing/Mux', [mdl '/DeltaK_Mux'], 'Inputs', '6', 'Position', [850 350 870 490]);
add_block('simulink/Sinks/Scope', [mdl '/Position_Scope'], 'Position', [1000 50 1050 90]);
add_block('simulink/Sinks/Scope', [mdl '/Attitude_Scope'], 'Position', [1000 100 1050 140]);

fprintf('  Phase B: Injecting MATLAB Function code...\n');
save_path_rl = fullfile(models_dir, [mdl '.slx']);
save_system(mdl, save_path_rl);
close_system(mdl, 0);
load_system(save_path_rl);
mfb_list = {[mdl '/Plant_6DOF/SixDOF_Dynamics'], sixdof_code; [mdl '/Thrust_Mixer'], mixer_code; [mdl '/Normalize_Obs'], normalize_code; [mdl '/Reward_Calculator'], reward_code; [mdl '/Crash_Detector'], crash_code};
rt = sfroot;
for k = 1:size(mfb_list, 1)
    bp = mfb_list{k,1}; cs = mfb_list{k,2};
    ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', bp);
    if ~isempty(ch); ch(1).Script = cs; fprintf('    OK: %s\n', bp);
    else; warning('Chart not found: %s', bp); end
end
save_system(mdl); close_system(mdl, 0); load_system(save_path_rl);

fprintf('  Phase C: Adding connections...\n');
add_line(plant, 'Forces_In/1', 'Wind_Sum/1');
add_line(plant, 'Wind_In/1', 'Wind_Sum/2');
add_line(plant, 'Wind_Sum/1', 'SixDOF_Dynamics/1');
add_line(plant, 'Moments_In/1', 'SixDOF_Dynamics/2');
add_line(plant, 'SixDOF_Dynamics/1', 'Pos_Out/1');
add_line(plant, 'SixDOF_Dynamics/2', 'Vel_Out/1');
add_line(plant, 'SixDOF_Dynamics/3', 'Euler_Out/1');
add_line(plant, 'SixDOF_Dynamics/4', 'Rates_Out/1');
add_line(mdl, 'Setpoint_X/1', 'Err_X/1');
add_line(mdl, 'Setpoint_Y/1', 'Err_Y/1');
add_line(mdl, 'Setpoint_Z/1', 'Err_Z/1');
add_line(mdl, 'Setpoint_Psi/1', 'Err_Psi/1');
add_line(mdl, 'Err_X/1', 'PID_X/1');
add_line(mdl, 'Err_Y/1', 'PID_Y/1');
add_line(mdl, 'Err_Z/1', 'PID_Z/1');
add_line(mdl, 'PID_X/1', 'Sat_Theta/1');
add_line(mdl, 'PID_Y/1', 'Sat_Phi/1');
add_line(mdl, 'Sat_Phi/1', 'Err_Phi/1');
add_line(mdl, 'Sat_Theta/1', 'Err_Theta/1');
add_line(mdl, 'Err_Phi/1', 'PID_Roll/1');
add_line(mdl, 'Err_Theta/1', 'PID_Pitch/1');
add_line(mdl, 'Err_Psi/1', 'PID_Yaw/1');
add_line(mdl, 'PID_Z/1', 'Thrust_Mixer/1');
add_line(mdl, 'PID_Roll/1', 'Thrust_Mixer/2');
add_line(mdl, 'PID_Pitch/1', 'Thrust_Mixer/3');
add_line(mdl, 'PID_Yaw/1', 'Thrust_Mixer/4');
add_line(mdl, 'Thrust_Mixer/1', 'Plant_6DOF/1');
add_line(mdl, 'Thrust_Mixer/2', 'Plant_6DOF/2');
add_line(mdl, 'Wind_Step_Fx/1', 'Wind_Mux/1');
add_line(mdl, 'Wind_Fy/1', 'Wind_Mux/2');
add_line(mdl, 'Wind_Fz/1', 'Wind_Mux/3');
add_line(mdl, 'Wind_Mux/1', 'Plant_6DOF/3');
add_line(mdl, 'Plant_6DOF/1', 'Pos_Demux/1');
add_line(mdl, 'Pos_Demux/1', 'Err_X/2');
add_line(mdl, 'Pos_Demux/2', 'Err_Y/2');
add_line(mdl, 'Pos_Demux/3', 'Err_Z/2');
add_line(mdl, 'Plant_6DOF/3', 'Euler_Demux/1');
add_line(mdl, 'Euler_Demux/1', 'Err_Phi/2');
add_line(mdl, 'Euler_Demux/2', 'Err_Theta/2');
add_line(mdl, 'Euler_Demux/3', 'Err_Psi/2');
add_line(mdl, 'Err_X/1', 'Obs_Mux/1');
add_line(mdl, 'Err_Y/1', 'Obs_Mux/2');
add_line(mdl, 'Err_Z/1', 'Obs_Mux/3');
add_line(mdl, 'Err_Phi/1', 'Obs_Mux/4');
add_line(mdl, 'Err_Theta/1', 'Obs_Mux/5');
add_line(mdl, 'Err_Psi/1', 'Obs_Mux/6');
add_line(mdl, 'Err_X/1', 'DerivFilter_1/1');
add_line(mdl, 'Err_Y/1', 'DerivFilter_2/1');
add_line(mdl, 'Err_Z/1', 'DerivFilter_3/1');
add_line(mdl, 'Err_Phi/1', 'DerivFilter_4/1');
add_line(mdl, 'Err_Theta/1', 'DerivFilter_5/1');
add_line(mdl, 'Err_Psi/1', 'DerivFilter_6/1');
for i = 1:6; add_line(mdl, sprintf('DerivFilter_%d/1', i), sprintf('Obs_Mux/%d', i+6)); end
add_line(mdl, 'Tap_Kp/1', 'Obs_Mux/13');
add_line(mdl, 'Tap_Ki/1', 'Obs_Mux/14');
add_line(mdl, 'Tap_Kd/1', 'Obs_Mux/15');
add_line(mdl, 'Obs_Mux/1', 'Normalize_Obs/1');
add_line(mdl, 'Normalize_Obs/1', 'RL_Agent/1');
add_line(mdl, 'Reward_Calculator/1', 'RL_Agent/2');
add_line(mdl, 'RL_Agent/1', 'Action_Demux/1');
for ch = 1:6; add_line(mdl, sprintf('Action_Demux/%d', ch), sprintf('Alpha_Scale_%d/1', ch)); end
add_line(mdl, 'Err_X/1', 'Err_Pos_Mux/1');
add_line(mdl, 'Err_Y/1', 'Err_Pos_Mux/2');
add_line(mdl, 'Err_Z/1', 'Err_Pos_Mux/3');
add_line(mdl, 'Err_Phi/1', 'Err_Att_Mux/1');
add_line(mdl, 'Err_Theta/1', 'Err_Att_Mux/2');
add_line(mdl, 'Err_Psi/1', 'Err_Att_Mux/3');
for ch = 1:6; add_line(mdl, sprintf('Alpha_Scale_%d/1', ch), sprintf('DeltaK_Mux/%d', ch)); end
add_line(mdl, 'Err_Pos_Mux/1', 'Reward_Calculator/1');
add_line(mdl, 'Err_Att_Mux/1', 'Reward_Calculator/2');
add_line(mdl, 'DeltaK_Mux/1', 'Reward_Calculator/3');
add_line(mdl, 'Pos_Demux/1', 'Crash_Detector/1');
add_line(mdl, 'Pos_Demux/2', 'Crash_Detector/2');
add_line(mdl, 'Pos_Demux/3', 'Crash_Detector/3');
add_line(mdl, 'Euler_Demux/1', 'Crash_Detector/4');
add_line(mdl, 'Euler_Demux/2', 'Crash_Detector/5');
add_line(mdl, 'Crash_Detector/1', 'Reward_Calculator/4');
add_line(mdl, 'Crash_Detector/1', 'Stop_Sim/1');
add_line(mdl, 'Plant_6DOF/1', 'Position_Scope/1');
add_line(mdl, 'Plant_6DOF/3', 'Attitude_Scope/1');
try; Simulink.BlockDiagram.arrangeSystem(mdl); catch; end
save_system(mdl);
fprintf('  RL model saved: %s\n', save_path_rl);

mdl_base = 'quadcopter_baseline_pid';
fprintf('Building %s.slx ...\n', mdl_base);
if bdIsLoaded(mdl_base); close_system(mdl_base, 0); end
new_system(mdl_base); open_system(mdl_base);
set_param(mdl_base, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', 'Ts_inner', 'StopTime', 'T_episode', 'SaveOutput', 'on', 'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on');
add_block('simulink/Sources/Constant', [mdl_base '/Setpoint_X'], 'Value', 'x_d', 'Position', [50 50 100 70]);
add_block('simulink/Sources/Constant', [mdl_base '/Setpoint_Y'], 'Value', 'y_d', 'Position', [50 120 100 140]);
add_block('simulink/Sources/Constant', [mdl_base '/Setpoint_Z'], 'Value', 'z_d', 'Position', [50 190 100 210]);
add_block('simulink/Sources/Constant', [mdl_base '/Setpoint_Psi'], 'Value', 'psi_d', 'Position', [50 260 100 280]);
pb = [mdl_base '/Plant_6DOF'];
add_block('simulink/Ports & Subsystems/Subsystem', pb, 'Position', [700 150 850 350]);
delete_line(pb, 'In1/1', 'Out1/1'); delete_block([pb '/In1']); delete_block([pb '/Out1']);
add_block('simulink/Sources/In1', [pb '/Forces_In'], 'Position', [30 80 60 100], 'Port', '1');
add_block('simulink/Sources/In1', [pb '/Moments_In'], 'Position', [30 160 60 180], 'Port', '2');
add_block('simulink/Sources/In1', [pb '/Wind_In'], 'Position', [30 240 60 260], 'Port', '3');
add_block('simulink/Math Operations/Add', [pb '/Wind_Sum'], 'Inputs', '++', 'Position', [150 80 180 110]);
add_block('simulink/User-Defined Functions/MATLAB Function', [pb '/SixDOF_Dynamics'], 'Position', [250 50 450 300]);
add_block('simulink/Sinks/Out1', [pb '/Pos_Out'], 'Position', [550 60 580 80], 'Port', '1');
add_block('simulink/Sinks/Out1', [pb '/Vel_Out'], 'Position', [550 120 580 140], 'Port', '2');
add_block('simulink/Sinks/Out1', [pb '/Euler_Out'], 'Position', [550 180 580 200], 'Port', '3');
add_block('simulink/Sinks/Out1', [pb '/Rates_Out'], 'Position', [550 240 580 260], 'Port', '4');
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl_base '/Thrust_Mixer'], 'Position', [500 180 620 280]);
add_block('simulink/Math Operations/Sum', [mdl_base '/Err_X'], 'Inputs', '+-', 'Position', [200 50 230 70]);
add_block('simulink/Math Operations/Sum', [mdl_base '/Err_Y'], 'Inputs', '+-', 'Position', [200 120 230 140]);
add_block('simulink/Math Operations/Sum', [mdl_base '/Err_Z'], 'Inputs', '+-', 'Position', [200 190 230 210]);
add_block('simulink/Math Operations/Sum', [mdl_base '/Err_Phi'], 'Inputs', '+-', 'Position', [200 350 230 370]);
add_block('simulink/Math Operations/Sum', [mdl_base '/Err_Theta'], 'Inputs', '+-', 'Position', [200 420 230 440]);
add_block('simulink/Math Operations/Sum', [mdl_base '/Err_Psi'], 'Inputs', '+-', 'Position', [200 490 230 510]);
add_block('simulink/Continuous/PID Controller', [mdl_base '/PID_X'], 'Position', [350 40 430 80], 'P', 'Kp_x', 'I', 'Ki_x', 'D', 'Kd_x', 'Controller', 'PID');
add_block('simulink/Continuous/PID Controller', [mdl_base '/PID_Y'], 'Position', [350 110 430 150], 'P', 'Kp_y', 'I', 'Ki_y', 'D', 'Kd_y', 'Controller', 'PID');
add_block('simulink/Continuous/PID Controller', [mdl_base '/PID_Z'], 'Position', [350 180 430 220], 'P', 'Kp_z', 'I', 'Ki_z', 'D', 'Kd_z', 'Controller', 'PID');
add_block('simulink/Discontinuities/Saturation', [mdl_base '/Sat_Theta'], 'UpperLimit', '0.3', 'LowerLimit', '-0.3', 'Position', [460 40 500 80]);
add_block('simulink/Discontinuities/Saturation', [mdl_base '/Sat_Phi'], 'UpperLimit', '0.3', 'LowerLimit', '-0.3', 'Position', [460 110 500 150]);
add_block('simulink/Continuous/PID Controller', [mdl_base '/PID_Roll'], 'Position', [380 340 460 380], 'P', 'Kp_phi', 'I', 'Ki_phi', 'D', 'Kd_phi', 'Controller', 'PID');
add_block('simulink/Continuous/PID Controller', [mdl_base '/PID_Pitch'], 'Position', [380 410 460 450], 'P', 'Kp_theta', 'I', 'Ki_theta', 'D', 'Kd_theta', 'Controller', 'PID');
add_block('simulink/Continuous/PID Controller', [mdl_base '/PID_Yaw'], 'Position', [380 480 460 520], 'P', 'Kp_psi', 'I', 'Ki_psi', 'D', 'Kd_psi', 'Controller', 'PID');
add_block('simulink/Sources/Step', [mdl_base '/Wind_Step_Fx'], 'Time', 'wind_onset_time', 'Before', '0', 'After', 'wind_force_N', 'Position', [500 500 550 520]);
add_block('simulink/Sources/Constant', [mdl_base '/Wind_Fy'], 'Value', '0', 'Position', [500 540 550 560]);
add_block('simulink/Sources/Constant', [mdl_base '/Wind_Fz'], 'Value', '0', 'Position', [500 580 550 600]);
add_block('simulink/Signal Routing/Mux', [mdl_base '/Wind_Mux'], 'Inputs', '3', 'Position', [590 500 610 600]);
add_block('simulink/Signal Routing/Demux', [mdl_base '/Pos_Demux'], 'Outputs', '3', 'Position', [900 150 920 210]);
add_block('simulink/Signal Routing/Demux', [mdl_base '/Euler_Demux'], 'Outputs', '3', 'Position', [900 250 920 310]);
add_block('simulink/Sinks/Scope', [mdl_base '/Position_Scope'], 'Position', [1000 50 1050 90]);
add_block('simulink/Sinks/Scope', [mdl_base '/Attitude_Scope'], 'Position', [1000 100 1050 140]);
save_path_base = fullfile(models_dir, [mdl_base '.slx']);
save_system(mdl_base, save_path_base);
close_system(mdl_base, 0); load_system(save_path_base);
rt2 = sfroot;
mfb_b = {[mdl_base '/Plant_6DOF/SixDOF_Dynamics'], sixdof_code; [mdl_base '/Thrust_Mixer'], mixer_code};
for k = 1:size(mfb_b,1)
    ch = rt2.find('-isa', 'Stateflow.EMChart', 'Path', mfb_b{k,1});
    if ~isempty(ch); ch(1).Script = mfb_b{k,2}; fprintf('    OK: %s\n', mfb_b{k,1}); end
end
save_system(mdl_base); close_system(mdl_base, 0); load_system(save_path_base);
add_line(pb, 'Forces_In/1', 'Wind_Sum/1');
add_line(pb, 'Wind_In/1', 'Wind_Sum/2');
add_line(pb, 'Wind_Sum/1', 'SixDOF_Dynamics/1');
add_line(pb, 'Moments_In/1', 'SixDOF_Dynamics/2');
add_line(pb, 'SixDOF_Dynamics/1', 'Pos_Out/1');
add_line(pb, 'SixDOF_Dynamics/2', 'Vel_Out/1');
add_line(pb, 'SixDOF_Dynamics/3', 'Euler_Out/1');
add_line(pb, 'SixDOF_Dynamics/4', 'Rates_Out/1');
add_line(mdl_base, 'Setpoint_X/1', 'Err_X/1');
add_line(mdl_base, 'Setpoint_Y/1', 'Err_Y/1');
add_line(mdl_base, 'Setpoint_Z/1', 'Err_Z/1');
add_line(mdl_base, 'Setpoint_Psi/1', 'Err_Psi/1');
add_line(mdl_base, 'Err_X/1', 'PID_X/1');
add_line(mdl_base, 'Err_Y/1', 'PID_Y/1');
add_line(mdl_base, 'Err_Z/1', 'PID_Z/1');
add_line(mdl_base, 'PID_X/1', 'Sat_Theta/1');
add_line(mdl_base, 'PID_Y/1', 'Sat_Phi/1');
add_line(mdl_base, 'Sat_Phi/1', 'Err_Phi/1');
add_line(mdl_base, 'Sat_Theta/1', 'Err_Theta/1');
add_line(mdl_base, 'Err_Phi/1', 'PID_Roll/1');
add_line(mdl_base, 'Err_Theta/1', 'PID_Pitch/1');
add_line(mdl_base, 'Err_Psi/1', 'PID_Yaw/1');
add_line(mdl_base, 'PID_Z/1', 'Thrust_Mixer/1');
add_line(mdl_base, 'PID_Roll/1', 'Thrust_Mixer/2');
add_line(mdl_base, 'PID_Pitch/1', 'Thrust_Mixer/3');
add_line(mdl_base, 'PID_Yaw/1', 'Thrust_Mixer/4');
add_line(mdl_base, 'Thrust_Mixer/1', 'Plant_6DOF/1');
add_line(mdl_base, 'Thrust_Mixer/2', 'Plant_6DOF/2');
add_line(mdl_base, 'Wind_Step_Fx/1', 'Wind_Mux/1');
add_line(mdl_base, 'Wind_Fy/1', 'Wind_Mux/2');
add_line(mdl_base, 'Wind_Fz/1', 'Wind_Mux/3');
add_line(mdl_base, 'Wind_Mux/1', 'Plant_6DOF/3');
add_line(mdl_base, 'Plant_6DOF/1', 'Pos_Demux/1');
add_line(mdl_base, 'Plant_6DOF/3', 'Euler_Demux/1');
add_line(mdl_base, 'Pos_Demux/1', 'Err_X/2');
add_line(mdl_base, 'Pos_Demux/2', 'Err_Y/2');
add_line(mdl_base, 'Pos_Demux/3', 'Err_Z/2');
add_line(mdl_base, 'Euler_Demux/1', 'Err_Phi/2');
add_line(mdl_base, 'Euler_Demux/2', 'Err_Theta/2');
add_line(mdl_base, 'Euler_Demux/3', 'Err_Psi/2');
add_line(mdl_base, 'Plant_6DOF/1', 'Position_Scope/1');
add_line(mdl_base, 'Plant_6DOF/3', 'Attitude_Scope/1');
try; Simulink.BlockDiagram.arrangeSystem(mdl_base); catch; end
save_system(mdl_base); close_system(mdl_base, 0);
close_system(mdl, 0);
fprintf('  Baseline saved: %s\n', save_path_base);
fprintf('\n=== Simulink Models Built Successfully ===\n');
end

function code = build_mixer_code()
NL = newline;
code = ['function [F_body, M_body] = Thrust_Mixer(U1, U2, U3, U4)' NL '%#codegen' NL 'F_body = [0; 0; -U1];' NL 'M_body = [U2; U3; U4];' NL];
end

function code = build_sixdof_code(mv, gv, Ixxv, Iyyv, Izzv, Axv, Ayv, Azv, Tsv)
NL = newline;
code = ['function [pos, vel_b, euler, rates] = SixDOF_Dynamics(F_body, M_body)' NL ...
'%#codegen' NL ...
'F_body=F_body(:); M_body=M_body(:);' NL ...
'persistent x_state' NL ...
'if isempty(x_state); x_state=zeros(12,1); x_state(3)=-1; end' NL ...
sprintf('m_p=%.6f; g_p=%.4f;\n', mv, gv) ...
sprintf('Ixx_p=%.6e; Iyy_p=%.6e; Izz_p=%.6e;\n', Ixxv, Iyyv, Izzv) ...
sprintf('Ax_p=%.4f; Ay_p=%.4f; Az_p=%.4f; Ts_p=%.6f;\n', Axv, Ayv, Azv, Tsv) ...
'u_s=x_state(4); v_s=x_state(5); w_s=x_state(6);' NL ...
'phi_s=x_state(7); theta_s=x_state(8); psi_s=x_state(9);' NL ...
'p_s=x_state(10); q_s=x_state(11); r_s=x_state(12);' NL ...
'cphi=cos(phi_s); sphi=sin(phi_s); cth=cos(theta_s); sth=sin(theta_s); cpsi=cos(psi_s); spsi=sin(psi_s);' NL ...
'R11=cth*cpsi; R12=sphi*sth*cpsi-cphi*spsi; R13=cphi*sth*cpsi+sphi*spsi;' NL ...
'R21=cth*spsi; R22=sphi*sth*spsi+cphi*cpsi; R23=cphi*sth*spsi-sphi*cpsi;' NL ...
'R31=-sth; R32=sphi*cth; R33=cphi*cth;' NL ...
'Fn=zeros(3,1);' NL ...
'Fn(1)=R11*F_body(1)+R12*F_body(2)+R13*F_body(3);' NL ...
'Fn(2)=R21*F_body(1)+R22*F_body(2)+R23*F_body(3);' NL ...
'Fn(3)=R31*F_body(1)+R32*F_body(2)+R33*F_body(3)+m_p*g_p;' NL ...
'db=[-Ax_p*u_s;-Ay_p*v_s;-Az_p*w_s];' NL ...
'Dn=zeros(3,1);' NL ...
'Dn(1)=R11*db(1)+R12*db(2)+R13*db(3);' NL ...
'Dn(2)=R21*db(1)+R22*db(2)+R23*db(3);' NL ...
'Dn(3)=R31*db(1)+R32*db(2)+R33*db(3);' NL ...
'an=zeros(3,1);' NL ...
'an(1)=(Fn(1)+Dn(1))/m_p; an(2)=(Fn(2)+Dn(2))/m_p; an(3)=(Fn(3)+Dn(3))/m_p;' NL ...
'ab=zeros(3,1);' NL ...
'ab(1)=R11*an(1)+R21*an(2)+R31*an(3);' NL ...
'ab(2)=R12*an(1)+R22*an(2)+R32*an(3);' NL ...
'ab(3)=R13*an(1)+R23*an(2)+R33*an(3);' NL ...
'Iw1=Ixx_p*p_s; Iw2=Iyy_p*q_s; Iw3=Izz_p*r_s;' NL ...
'alr=zeros(3,1);' NL ...
'alr(1)=(M_body(1)-(q_s*Iw3-r_s*Iw2))/Ixx_p;' NL ...
'alr(2)=(M_body(2)-(r_s*Iw1-p_s*Iw3))/Iyy_p;' NL ...
'alr(3)=(M_body(3)-(p_s*Iw2-q_s*Iw1))/Izz_p;' NL ...
'cth_s=cth; if abs(cth_s)<1e-6; cth_s=1e-6; end' NL ...
'ed=zeros(3,1);' NL ...
'ed(1)=p_s+(sphi*sth/cth_s)*q_s+(cphi*sth/cth_s)*r_s;' NL ...
'ed(2)=cphi*q_s-sphi*r_s;' NL ...
'ed(3)=(sphi/cth_s)*q_s+(cphi/cth_s)*r_s;' NL ...
'vn=zeros(3,1);' NL ...
'vn(1)=R11*u_s+R12*v_s+R13*w_s;' NL ...
'vn(2)=R21*u_s+R22*v_s+R23*w_s;' NL ...
'vn(3)=R31*u_s+R32*v_s+R33*w_s;' NL ...
'x_state(1)=x_state(1)+vn(1)*Ts_p;' NL ...
'x_state(2)=x_state(2)+vn(2)*Ts_p;' NL ...
'x_state(3)=x_state(3)+vn(3)*Ts_p;' NL ...
'x_state(4)=x_state(4)+ab(1)*Ts_p;' NL ...
'x_state(5)=x_state(5)+ab(2)*Ts_p;' NL ...
'x_state(6)=x_state(6)+ab(3)*Ts_p;' NL ...
'x_state(7)=x_state(7)+ed(1)*Ts_p;' NL ...
'x_state(8)=x_state(8)+ed(2)*Ts_p;' NL ...
'x_state(9)=x_state(9)+ed(3)*Ts_p;' NL ...
'x_state(10)=x_state(10)+alr(1)*Ts_p;' NL ...
'x_state(11)=x_state(11)+alr(2)*Ts_p;' NL ...
'x_state(12)=x_state(12)+alr(3)*Ts_p;' NL ...
'pos=x_state(1:3); vel_b=x_state(4:6); euler=x_state(7:9); rates=x_state(10:12);' NL];
end

function code = build_normalize_code(np, na, nv, nr, nkp, nki, nkd)
NL = newline;
code = ['function obs_norm = Normalize_Obs(obs_raw)' NL '%#codegen' NL ...
'obs_raw=obs_raw(:); obs_norm=zeros(15,1);' NL ...
sprintf('obs_norm(1:3)=obs_raw(1:3)/%.1f;\n', np) ...
sprintf('obs_norm(4:6)=obs_raw(4:6)/%.1f;\n', na) ...
sprintf('obs_norm(7:9)=obs_raw(7:9)/%.1f;\n', nv) ...
sprintf('obs_norm(10:12)=obs_raw(10:12)/%.1f;\n', nr) ...
sprintf('obs_norm(13)=obs_raw(13)/%.1f;\n', nkp) ...
sprintf('obs_norm(14)=obs_raw(14)/%.1f;\n', nki) ...
sprintf('obs_norm(15)=obs_raw(15)/%.1f;\n', nkd) ...
'obs_norm=max(min(obs_norm,1),-1);' NL];
end

function code = build_reward_code()
NL = newline;
code = ['function reward = Reward_Calculator(e_pos, e_att, delta_K, crashed)' NL '%#codegen' NL ...
'e_pos=e_pos(:); e_att=e_att(:); delta_K=delta_K(:);' NL ...
'pc=0; for i=1:3; pc=pc+e_pos(i)*e_pos(i); end' NL ...
'ac=0; for i=1:3; ac=ac+e_att(i)*e_att(i); end' NL ...
'gc=0; for i=1:numel(delta_K); gc=gc+delta_K(i)*delta_K(i); end' NL ...
'pc=1.0*pc; ac=0.5*ac; gc=0.1*gc;' NL ...
'bonus=0; if pc<0.05 && ac<0.02; bonus=0.3; end' NL ...
'cc=double(crashed>0.5)*100.0;' NL ...
'reward=-(pc+ac+gc)+bonus-cc;' NL ...
'reward=max(min(reward,10.0),-110.0);' NL];
end

function code = build_crash_code()
NL = newline;
code = ['function crashed = Crash_Detector(x, y, z, phi, theta)' NL '%#codegen' NL ...
'pd=sqrt(x*x+y*y+(z-1)*(z-1));' NL ...
'ad=sqrt(phi*phi+theta*theta);' NL ...
'crashed=double(pd>3.0 || ad>1.2);' NL];
end
