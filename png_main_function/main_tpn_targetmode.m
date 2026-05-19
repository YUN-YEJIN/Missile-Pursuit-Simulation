clc; clear; close all;

%% ================================
%  1. Simulation setting
% ================================
dt = 0.01;
T_end = 1000;
time = 0:dt:T_end;

encounter_radius = 10;   % general encounter condition [m]

%% ================================
%  2. Chaser initial condition
% ================================
C_pos = [0; 0];
C_speed = 30;             % 기존 코드 값 유지. 필요하면 300 등으로 수정.
C_heading_deg = 20;
C_heading = deg2rad(C_heading_deg);

C_vel = C_speed * [cos(C_heading); sin(C_heading)];

%% ================================
%  3. Target initial condition input
% ================================
disp("=== Target Initial Condition Input ===");

Tx = input("Target initial x position [m] = ");
Ty = input("Target initial y position [m] = ");
T_speed = input("Target speed [m/s] = ");
T_heading_deg = input("Target heading angle [deg] = ");

T_pos = [Tx; Ty];
T_heading = deg2rad(T_heading_deg);
T_vel = T_speed * [cos(T_heading); sin(T_heading)];

%% ================================
%  4. Target mode / target state / target config
% ================================
[target_mode, use_random_switching, mode_switch_steps, ...
    targetState, targetConfig] = targetModeFunction("setup", dt);

%% ================================
%  5. Chaser guidance parameters
% ================================
N = 3;
a_max = 120;

%% ================================
%  6. Data save
% ================================
C_hist = zeros(2, length(time));
T_hist = zeros(2, length(time));
R_hist = zeros(1, length(time));
a_hist = zeros(1, length(time));
mode_hist = zeros(1, length(time));
panic_hist = zeros(1, length(time));

success = false;

%% ================================
%  7. Realtime animation setting
% ================================
plot_update_interval = 0.1;   % [s]
plotData = plotFunction("initRealtime", C_pos, T_pos, dt, plot_update_interval);

%% ================================
%  8. Main simulation loop
% ================================
for k = 1:length(time)

    t_now = time(k);

    % random mode switching
    target_mode = targetModeFunction("switch", ...
        k, use_random_switching, target_mode, mode_switch_steps);

    % Save current position
    C_hist(:, k) = C_pos;
    T_hist(:, k) = T_pos;
    mode_hist(k) = target_mode;

    % Relative position and velocity
    r = T_pos - C_pos;
    v_rel = T_vel - C_vel;

    % Distance
    R = norm(r);
    R_hist(k) = R;

    % Check encounter
    if R < encounter_radius
        fprintf("Encounter success at t = %.2f sec\n", t_now);

        C_hist = C_hist(:, 1:k);
        T_hist = T_hist(:, 1:k);
        R_hist = R_hist(1:k);
        a_hist = a_hist(1:k);
        mode_hist = mode_hist(1:k);
        panic_hist = panic_hist(1:k);
        time = time(1:k);

        success = true;
        break;
    end

    % LOS rate
    lambda_dot = (r(1)*v_rel(2) - r(2)*v_rel(1)) / R^2;

    % Closing velocity
    Vc = -dot(r, v_rel) / R;

    % PN-like guidance
    % 기존 코드의 if Vc < 0 로직은 abs(Vc)와 동일하게 정리 가능
    a_cmd = N * abs(Vc) * lambda_dot;

    % Acceleration limit
    a_cmd = max(min(a_cmd, a_max), -a_max);
    a_hist(k) = a_cmd;

    % Lateral acceleration direction
    C_speed_now = norm(C_vel);
    normal_dir = [-C_vel(2); C_vel(1)] / C_speed_now;

    % Chaser acceleration
    a_C = a_cmd * normal_dir;

    % Update chaser
    C_vel = C_vel + a_C * dt;
    C_vel = C_speed * C_vel / norm(C_vel);   % constant speed
    C_pos = C_pos + C_vel * dt;

    % Update target according to mode
    [T_pos, T_vel, T_heading, targetState] = targetModeFunction("update", ...
        T_pos, T_speed, T_heading, dt, t_now, target_mode, ...
        targetState, targetConfig, C_pos);

    panic_hist(k) = double(targetState.panic_active);

    % Realtime plot update
    plotData = plotFunction("updateRealtime", plotData, k, C_hist, T_hist);
end

if ~success
    fprintf("Simulation ended without encounter.\n");
    last_idx = find(R_hist > 0, 1, 'last');
    fprintf("Final range = %.2f m\n", R_hist(last_idx));
end

%% ================================
%  9. Result plots
% ================================
plotFunction("summary", time, C_hist, T_hist, R_hist, a_hist, mode_hist, panic_hist);
