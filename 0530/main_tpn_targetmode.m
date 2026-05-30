clc; clear; close all;

%% ================================
%  1. Simulation setting
% ================================
dt = 0.01;
T_end = 300;
time = 0:dt:T_end;

encounter_radius = 10;   % general encounter condition [m]

pp_count = 0;
pn_count = 0;

%% ================================
%  3. Target initial condition input
% ================================
disp("=== Target Initial Condition Input ===");

Tx          = input("Target initial x position [m] = ");
Ty          = input("Target initial y position [m] = ");
T_speed     = input("Target speed [m/s] = ");
T_heading_deg = input("Target heading angle [deg] = ");

T_pos     = [Tx; Ty];
T_heading = deg2rad(T_heading_deg);
T_vel     = T_speed * [cos(T_heading); sin(T_heading)];
T_goal = [10000; 1000];

%% ================================
%  2. Chaser initial condition
%  (T_pos 정의 후에 LOS 방향으로 heading 설정)
% ================================
C_pos     = [0; 0];
C_speed   = 300;
%C_heading_deg = 20;
%C_heading = deg2rad(C_heading_deg);

initial_los = T_pos - C_pos;
C_heading   = atan2(initial_los(2), initial_los(1));  % LOS 방향으로 출발
C_vel       = C_speed * [cos(C_heading); sin(C_heading)];

%% ================================
%  4. Target mode / target state / target config
% ================================
[target_mode, targetState, targetConfig] = TPN_Function("setup", dt);
targetState.just_updated_turn = false;

%% ================================
%  5. Chaser guidance parameters
% ================================
N     = 3;
a_max = 120;

%% ================================
%  6. Data save
% ================================
C_hist    = zeros(2, length(time));
T_hist    = zeros(2, length(time));
R_hist    = zeros(1, length(time));
a_hist    = zeros(1, length(time));
mode_hist = zeros(1, length(time));
guide_hist = zeros(1, length(time));   % PurePursuit=1, PNG=2
panic2_log = zeros(0, 2); 
panic3_log = zeros(0, 2); 
panic4_log = zeros(0, 2); 
success = false;

%% ================================
%  7. Realtime animation setting
% ================================
plot_update_interval = 0.1;   % [s]
plotData = plotFunction("initRealtime", C_pos, T_pos, T_goal, dt, plot_update_interval);

%% ================================
%  8. Main simulation loop
% ================================
for k = 1:length(time)

    t_now = time(k);

    % Save current position
    C_hist(:, k) = C_pos;
    T_hist(:, k) = T_pos;
    mode_hist(k) = target_mode;

    % Relative position and velocity
    r     = T_pos - C_pos;
    v_rel = T_vel - C_vel;

    % Distance
    R = norm(r);
    R_hist(k) = R;

     % 1) 요격 성공 여부 먼저 검사 (Encounter check)
    if R < encounter_radius
        fprintf("Encounter success at t = %.2f sec\n", t_now);

        C_hist     = C_hist(:, 1:k);
        T_hist     = T_hist(:, 1:k);
        R_hist     = R_hist(1:k);
        a_hist     = a_hist(1:k);
        mode_hist  = mode_hist(1:k);
        guide_hist = guide_hist(1:k);
        time       = time(1:k);

        success = true;
        break;
    end

    % LOS rate
    lambda_dot = (r(1)*v_rel(2) - r(2)*v_rel(1)) / R^2;

    % Closing velocity
    Vc = -dot(r, v_rel) / R;

    % ================================
    % Hybrid guidance: PN + Pure Pursuit fallback
    % ================================

    % 1) PN command (Vc > 0 일 때만)
    a_pn = N * max(Vc, 0) * lambda_dot;

    % 2) Pure Pursuit command
    r_hat = r / R;
    v_hat = C_vel / norm(C_vel);

    heading_error = atan2( ...
        v_hat(1)*r_hat(2) - v_hat(2)*r_hat(1), ...
        dot(v_hat, r_hat));

    K_heading = 2.0;   % heading correction gain [1/s]
    a_pp = K_heading * C_speed * heading_error;

    % 3) Switching condition
    target_is_behind = dot(v_hat, r_hat) < 0;

    if Vc <= 0 || target_is_behind
        a_cmd    = a_pp;        % 뒤에 있거나 멀어지는 중 → Pure Pursuit
        pp_count = pp_count + 1;
        guide_hist(k) = 1;    
    else
        a_cmd    = a_pn;        % 가까워지는 중 → PN
        pn_count = pn_count + 1;
        guide_hist(k) = 2;     
    end

    % Acceleration limit
    a_cmd    = max(min(a_cmd, a_max), -a_max);
    a_hist(k)= a_cmd;

    % Lateral acceleration direction (속도벡터 수직 = TPN)
    normal_dir = [-C_vel(2); C_vel(1)] / norm(C_vel);

    % Chaser update
    a_C   = a_cmd * normal_dir;
    C_vel = C_vel + a_C * dt;
    C_vel = C_speed * C_vel / norm(C_vel);  % constant speed
    C_pos = C_pos + C_vel * dt;

    safety_margin_factor = 5.0;
    min_radius = 15.0;

    goal_rel  = T_goal - T_pos;
    goal_dist = norm(goal_rel);
    acceptance_radius = max(T_speed * dt * safety_margin_factor, min_radius);

    if goal_dist <= acceptance_radius
        disp("=================================");
        fprintf('Target이 목적지에 무사히 도달하여 시뮬레이션을 종료합니다! (t = %.2f초)\n', t_now);
        disp("=================================");

        % 현재까지의 데이터 크기로 축소
        C_hist     = C_hist(:, 1:k);
        T_hist     = T_hist(:, 1:k);
        R_hist     = R_hist(1:k);
        a_hist     = a_hist(1:k);
        mode_hist  = mode_hist(1:k);
        guide_hist = guide_hist(1:k);
        time       = time(1:k);
        
        success = false; % 요격 실패 상태로 간주
        break;
    else
        base_heading = atan2(goal_rel(2), goal_rel(1));
    end

    % Target update
    [T_pos, T_vel, T_heading, targetState] = TPN_Function("update", ...
        T_pos, T_speed, T_heading, base_heading, dt, t_now, target_mode, ...
        targetState, targetConfig, C_pos, C_vel, C_speed);

    if targetState.just_updated_turn
        if target_mode == 2
            panic2_log(end+1, :) = T_pos';
        elseif target_mode == 3
            panic3_log(end+1, :) = T_pos';
        elseif target_mode == 4
            panic4_log(end+1, :) = T_pos';
        end
    end

    % Realtime plot update
    plotData = plotFunction("updateRealtime", plotData, k, C_hist, T_hist, target_mode, T_heading, C_heading, panic2_log, panic3_log, panic4_log);
end

if ~success && goal_dist > acceptance_radius
    fprintf("Simulation ended without encounter.\n");
    last_idx = find(R_hist > 0, 1, 'last');
    fprintf("Final range = %.2f m\n", R_hist(last_idx));
end

fprintf("\nPN  used: %d steps\n", pn_count);
fprintf("PP  used: %d steps\n", pp_count);

%% ================================
%  9. Result plots
% ================================
plotFunction("summary", time, C_hist, T_hist, R_hist, a_hist, mode_hist, T_goal, guide_hist, panic2_log, panic3_log, panic4_log);