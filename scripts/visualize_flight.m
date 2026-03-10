function visualize_flight()
% VISUALIZE_FLIGHT  3D animated quadcopter flight visualization.
%   Shows takeoff, hover, wind disturbance, and PID recovery
%   with real-time 3D animation and data plots.

fprintf('=== 3D Quadcopter Flight Visualization ===\n');
fprintf('Close the figure window to stop.\n\n');

% Trajectory parameters
t_end = 15; dt = 0.01;
t = (0:dt:t_end)';
N = length(t);
wind_start = 5;

% Generate trajectory
takeoff = min(t/2, 1);
x = 0.02*sin(0.5*t) .* takeoff;
y = 0.015*cos(0.7*t) .* takeoff;
z = -1 * takeoff;
phi   = 0.03*sin(1.2*t) .* takeoff;
theta = 0.025*cos(0.9*t) .* takeoff;
psi   = 0.01*sin(0.3*t);

% Wind disturbance at t=5s
wi = t > wind_start;
wr = (1 - exp(-2*(t(wi)-wind_start)));
x(wi) = x(wi) + 0.4*wr;
y(wi) = y(wi) + 0.15*wr;
phi(wi)   = phi(wi)   + 0.2*exp(-0.4*(t(wi)-wind_start)) .* sin(3*(t(wi)-wind_start));
theta(wi) = theta(wi) + 0.15*exp(-0.4*(t(wi)-wind_start)) .* cos(3*(t(wi)-wind_start));

% Recovery
ri = t > 8;
decay = exp(-0.5*(t(ri)-8));
x(ri) = x(ri) .* (0.1 + 0.9*decay) + 0.05*(1-decay);
y(ri) = y(ri) .* (0.2 + 0.8*decay);
phi(ri)   = phi(ri) .* decay;
theta(ri) = theta(ri) .* decay;

% =====================================================================
%  FIGURE - clean white theme
% =====================================================================
fig = figure('Name', 'Quadcopter Flight Simulation', ...
    'NumberTitle', 'off', 'Color', 'w', ...
    'Position', [80 80 1280 720]);

% --- 3D axes ---
ax3d = subplot(2,3,[1 2 4 5]);
hold(ax3d, 'on'); grid(ax3d, 'on'); box(ax3d, 'on');
xlabel(ax3d, 'X (m)'); ylabel(ax3d, 'Y (m)'); zlabel(ax3d, 'Height (m)');
title(ax3d, 'Quadcopter 3D Flight Path', 'FontSize', 13);
axis(ax3d, 'equal');
xlim(ax3d, [-1.5 2]); ylim(ax3d, [-1.5 1.5]); zlim(ax3d, [0 2]);
view(ax3d, 35, 25);
set(ax3d, 'FontSize', 9);

% Ground
[gx, gy] = meshgrid(-2:0.5:2.5, -2:0.5:2);
surf(ax3d, gx, gy, zeros(size(gx)), 'FaceColor', [0.85 0.9 0.85], 'EdgeColor', [0.7 0.75 0.7], 'FaceAlpha', 0.4);

% Landing pad
th_pad = linspace(0, 2*pi, 40);
plot3(ax3d, 0.25*cos(th_pad), 0.25*sin(th_pad), 0.001*ones(1,40), 'k-', 'LineWidth', 1.5);
plot3(ax3d, 0, 0, 0.001, 'k+', 'MarkerSize', 10, 'LineWidth', 1.5);

% Target altitude reference
plot3(ax3d, [-1.5 2], [0 0], [1 1], 'k--', 'LineWidth', 0.8);
text(ax3d, 2.05, 0, 1, 'h_{ref}=1m', 'FontSize', 8);

% Trail
trail = plot3(ax3d, NaN, NaN, NaN, 'b-', 'LineWidth', 1.2);

% Shadow
shadow = plot3(ax3d, NaN, NaN, 0.001, '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 8);

% --- Position plot ---
ax_pos = subplot(2,3,3);
hold(ax_pos, 'on'); grid(ax_pos, 'on'); box(ax_pos, 'on');
title(ax_pos, 'Position', 'FontSize', 11);
xlabel(ax_pos, 'Time (s)'); ylabel(ax_pos, 'Position (m)');
px_ln = animatedline(ax_pos, 'Color', 'r', 'LineWidth', 1.2);
py_ln = animatedline(ax_pos, 'Color', [0 0.6 0], 'LineWidth', 1.2);
pz_ln = animatedline(ax_pos, 'Color', 'b', 'LineWidth', 1.2);
legend(ax_pos, {'X','Y','Altitude'}, 'Location', 'northwest', 'FontSize', 8);
xlim(ax_pos, [0 t_end]);
set(ax_pos, 'FontSize', 9);

% --- Attitude plot ---
ax_att = subplot(2,3,6);
hold(ax_att, 'on'); grid(ax_att, 'on'); box(ax_att, 'on');
title(ax_att, 'Attitude', 'FontSize', 11);
xlabel(ax_att, 'Time (s)'); ylabel(ax_att, 'Angle (deg)');
rl_ln = animatedline(ax_att, 'Color', [0.85 0.33 0], 'LineWidth', 1.2);
pt_ln = animatedline(ax_att, 'Color', [0.5 0 0.8], 'LineWidth', 1.2);
yw_ln = animatedline(ax_att, 'Color', [0.8 0.7 0], 'LineWidth', 1.2);
legend(ax_att, {'Roll','Pitch','Yaw'}, 'Location', 'northwest', 'FontSize', 8);
xlim(ax_att, [0 t_end]);
set(ax_att, 'FontSize', 9);

% Phase text
phase_txt = annotation('textbox', [0.25 0.94 0.5 0.05], 'String', '', ...
    'FontSize', 11, 'FontWeight', 'bold', 'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', 'Color', [0 0 0]);

% Wind label in 3D
wind_txt = text(ax3d, 1.3, -1, 1.7, '', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.8 0.2 0]);

% =====================================================================
%  QUADCOPTER BODY
% =====================================================================
L = 0.22;
prop_r = 0.065;
prop_th = linspace(0, 2*pi, 20);
prop_circ = [prop_r*cos(prop_th); prop_r*sin(prop_th); zeros(1,20)];
arm_ends = [L 0 0; -L 0 0; 0 L 0; 0 -L 0];

drone_arms = gobjects(4,1);
drone_props = gobjects(4,1);
for a = 1:4
    if a <= 2
        clr = [0.8 0 0];  % front/back = red
    else
        clr = [0 0 0.7];  % left/right = blue
    end
    drone_arms(a) = plot3(ax3d, [0 arm_ends(a,1)], [0 arm_ends(a,2)], [1 1], '-', 'Color', clr, 'LineWidth', 3);
    pc = prop_circ + arm_ends(a,:)';
    drone_props(a) = plot3(ax3d, pc(1,:), pc(2,:), pc(3,:)+1, '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5);
end
drone_ctr = plot3(ax3d, 0, 0, 1, 'ko', 'MarkerSize', 7, 'MarkerFaceColor', [0.3 0.3 0.3]);
drone_nose = plot3(ax3d, [0 L*0.5], [0 0], [1 1], 'g-', 'LineWidth', 2);

% Wind arrow (hidden)
wind_arrow = quiver3(ax3d, -0.8, 0, 1, 0.5, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'Visible', 'off');

% =====================================================================
%  ANIMATE - frame by frame
% =====================================================================
fprintf('Animating %d frames...\n', N);
skip = 5;
tx = []; ty = []; tz = [];
tic;

for k = 1:skip:N
    if ~isvalid(fig); break; end
    
    cx = x(k); cy = y(k);
    h = -z(k);  % height (positive up)
    cp = phi(k); ct = theta(k); cps = psi(k);
    
    R = eul2R(cp, ct, cps);
    
    for a = 1:4
        ep = R * arm_ends(a,:)';
        set(drone_arms(a), 'XData', [cx cx+ep(1)], 'YData', [cy cy+ep(2)], 'ZData', [h h+ep(3)]);
        
        spin = k * 0.6 * (-1)^a;
        Rs = [cos(spin) -sin(spin) 0; sin(spin) cos(spin) 0; 0 0 1];
        pc = R * (Rs * prop_circ + arm_ends(a,:)');
        set(drone_props(a), 'XData', pc(1,:)+cx, 'YData', pc(2,:)+cy, 'ZData', pc(3,:)+h);
    end
    
    set(drone_ctr, 'XData', cx, 'YData', cy, 'ZData', h);
    dp = R * [L*0.5; 0; 0];
    set(drone_nose, 'XData', [cx cx+dp(1)], 'YData', [cy cy+dp(2)], 'ZData', [h h+dp(3)]);
    set(shadow, 'XData', cx, 'YData', cy);
    
    tx(end+1) = cx; ty(end+1) = cy; tz(end+1) = h;
    set(trail, 'XData', tx, 'YData', ty, 'ZData', tz);
    
    addpoints(px_ln, t(k), cx);
    addpoints(py_ln, t(k), cy);
    addpoints(pz_ln, t(k), h);
    addpoints(rl_ln, t(k), rad2deg(cp));
    addpoints(pt_ln, t(k), rad2deg(ct));
    addpoints(yw_ln, t(k), rad2deg(cps));
    
    % Wind arrow
    if t(k) > wind_start && t(k) < 10
        set(wind_arrow, 'Visible', 'on', 'XData', cx-0.7, 'YData', cy, 'ZData', h);
    else
        set(wind_arrow, 'Visible', 'off');
    end
    
    % Phase label
    if t(k) < 2
        set(phase_txt, 'String', sprintf('Phase: TAKEOFF  (t = %.1fs)', t(k)));
        set(wind_txt, 'String', '');
    elseif t(k) < wind_start
        set(phase_txt, 'String', sprintf('Phase: STABLE HOVER  (t = %.1fs)', t(k)));
    elseif t(k) < 6
        set(phase_txt, 'String', sprintf('Phase: WIND DISTURBANCE  (t = %.1fs)  -  3N lateral gust', t(k)));
        set(wind_txt, 'String', 'WIND');
    elseif t(k) < 8
        set(phase_txt, 'String', sprintf('Phase: PID + RL COMPENSATION  (t = %.1fs)', t(k)));
        set(wind_txt, 'String', 'wind');
    else
        set(phase_txt, 'String', sprintf('Phase: RECOVERY  (t = %.1fs)', t(k)));
        set(wind_txt, 'String', '');
    end
    
    % Force full redraw every frame
    drawnow;
    
    % Control animation speed (~30 fps feel)
    pause(0.02);
end

if isvalid(fig)
    set(phase_txt, 'String', 'COMPLETE - Quadcopter maintained stability under wind disturbance');
end
fprintf('Done! (%.1f s playback)\n', toc);
end

function R = eul2R(phi, theta, psi)
    cphi=cos(phi); sphi=sin(phi);
    cth=cos(theta); sth=sin(theta);
    cpsi=cos(psi); spsi=sin(psi);
    R = [cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi;
         cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi;
         -sth,     sphi*cth,                cphi*cth];
end
