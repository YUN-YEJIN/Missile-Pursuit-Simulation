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
C_speed = 30;
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
%  4. Target behavior mode
% ================================
disp("=== Target Mode Selection ===");
disp("1 : Straight");
disp("2 : Evasive");
disp("3 : Random");
disp("4 : Reactive");
disp("5 : Panic Escape");
disp("0 : Random switching among 1~5");

target_mode_input = input("Select target mode = ");

if target_mode_input == 0
    use_random_switching = true;
    target_mode = randi(5);
else
    use_random_switching = false;
    target_mode = target_mode_input;
end

mode_switch_interval = 3.0;                  % [s]
mode_switch_steps = round(mode_switch_interval / dt);

%% ================================
%  5. Target state / config
% ================================
targetState.turn_sign = 1;
targetState.last_turn_flip_time = 0;
targetState.last_random_update_time = 0;
targetState.random_turn_rate = 0;

% panic state machine
targetState.panic_active = false;
targetState.panic_start_time = -inf;

targetConfig.evasive_turn_rate_deg = 18;
targetConfig.evasive_flip_interval = 1.2;

targetConfig.random_turn_rate_max_deg = 30;
targetConfig.random_update_interval = 1.0;

targetConfig.reactive_turn_rate_deg = 28;
targetConfig.reactive_range = 700;

% Mode 5: Panic escape config
targetConfig.panic_range = 700;          % enter panic if chaser is closer than this
targetConfig.recover_range = 850;        % leave panic only after this distance
targetConfig.panic_turn_rate_deg = 85;   % sharper than reactive
targetConfig.panic_hold_time = 1.0;      % minimum panic duration [s]

%% ================================
%  6. Chaser guidance parameters
% ================================
N = 3;
a_max = 120;

%% ================================
%  7. Data save
% ================================
C_hist = zeros(2, length(time));
T_hist = zeros(2, length(time));
R_hist = zeros(1, length(time));
a_hist = zeros(1, length(time));
mode_hist = zeros(1, length(time));
panic_hist = zeros(1, length(time));

success = false;

%% ================================
%  7-1. Realtime animation setting
% ================================
plot_update_interval = 0.1;                         % [s]
plot_update_steps = max(1, round(plot_update_interval / dt));

fig_anim = figure;
hold on; grid on; axis equal;
xlabel('X Position [m]');
ylabel('Y Position [m]');
title('2D Chaser-Target Realtime Demo');

% start markers
hC_start = plot(C_pos(1), T_pos(2)*0 + C_pos(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
hT_start = plot(T_pos(1), T_pos(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);

% trajectory line handles
hC_traj = plot(C_pos(1), C_pos(2), 'b', 'LineWidth', 2);
hT_traj = plot(T_pos(1), T_pos(2), 'r', 'LineWidth', 2);

% current position markers
hC_now = plot(C_pos(1), C_pos(2), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
hT_now = plot(T_pos(1), T_pos(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);

legend([hC_traj, hT_traj, hC_start, hT_start, hC_now, hT_now], ...
       {'Chaser trajectory', 'Target trajectory', ...
        'Chaser start', 'Target start', ...
        'Chaser current', 'Target current'}, ...
       'Location', 'best');

drawnow;

%% ================================
%  8. Main simulation loop
% ================================
for k = 1:length(time)

    t_now = time(k);

    % random mode switching
    if use_random_switching
        if k == 1 || mod(k-1, mode_switch_steps) == 0
            target_mode = randi(5);
        end
    end

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

    % Simple PN-like guidance
    if Vc < 0
        a_cmd = N * (-Vc) * lambda_dot;
    else
        a_cmd = N * Vc * lambda_dot;
    end

    
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
    [T_pos, T_vel, T_heading, targetState] = updateTargetMotion( ...
        T_pos, T_speed, T_heading, dt, t_now, target_mode, ...
        targetState, targetConfig, C_pos);

    panic_hist(k) = double(targetState.panic_active);
    % Realtime plot update
    if mod(k-1, plot_update_steps) == 0 || k == length(time)
        set(hC_traj, 'XData', C_hist(1,1:k), 'YData', C_hist(2,1:k));
        set(hT_traj, 'XData', T_hist(1,1:k), 'YData', T_hist(2,1:k));

        set(hC_now, 'XData', C_hist(1,k), 'YData', C_hist(2,k));
        set(hT_now, 'XData', T_hist(1,k), 'YData', T_hist(2,k));

        % 보기 좋게 축 자동 확장
        x_all = [C_hist(1,1:k), T_hist(1,1:k)];
        y_all = [C_hist(2,1:k), T_hist(2,1:k)];

        x_min = min(x_all); x_max = max(x_all);
        y_min = min(y_all); y_max = max(y_all);

        x_margin = max(50, 0.05 * max(1, x_max - x_min));
        y_margin = max(50, 0.05 * max(1, y_max - y_min));

        xlim([x_min - x_margin, x_max + x_margin]);
        ylim([y_min - y_margin, y_max + y_margin]);

        drawnow limitrate nocallbacks;
    end
end

if ~success
    fprintf("Simulation ended without encounter.\n");
    last_idx = find(R_hist > 0, 1, 'last');
    fprintf("Final range = %.2f m\n", R_hist(last_idx));
end

% %% ================================
% %  9. Plot trajectory
% % ================================
% figure;
% plot(C_hist(1,:), C_hist(2,:), 'b', 'LineWidth', 2); hold on;
% plot(T_hist(1,:), T_hist(2,:), 'r', 'LineWidth', 2);
% 
% plot(C_hist(1,1), C_hist(2,1), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
% plot(T_hist(1,1), T_hist(2,1), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
% 
% plot(C_hist(1,end), C_hist(2,end), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
% plot(T_hist(1,end), T_hist(2,end), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
% 
% grid on;
% axis equal;
% xlabel('X Position [m]');
% ylabel('Y Position [m]');
% legend('Chaser trajectory', 'Target trajectory', ...
%        'Chaser start', 'Target start', ...
%        'Chaser end', 'Target end');
% title('2D Chaser-Target Demo');

%% ================================
% 10. Plot range
% ================================
figure;
plot(time, R_hist, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Range [m]');
title('Distance between Chaser and Target');

%% ================================
% 11. Plot acceleration command
% ================================
figure;
plot(time, a_hist, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Acceleration Command [m/s^2]');
title('PN-like Acceleration Command');

%% ================================
% 12. Plot target mode history
% ================================
figure;
stairs(time, mode_hist, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Target Mode');
yticks([1 2 3 4 5]);
ylim([0.5 5.5]);
title('Target Behavior Mode History');

%% ================================
% 13. Plot panic state history
% ================================
figure;
stairs(time, panic_hist, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Panic Active');
yticks([0 1]);
ylim([-0.2 1.2]);
title('Panic Escape State');

%% ================================
% Local function
% ================================
function [T_pos, T_vel, T_heading, targetState] = updateTargetMotion( ...
    T_pos, T_speed, T_heading, dt, t_now, target_mode, ...
    targetState, targetConfig, chaser_pos)

    switch target_mode

        case 1
            % Straight
            % heading 유지
            % nothing to do

        case 2
            % Evasive: 좌우 weaving
            flip_interval = targetConfig.evasive_flip_interval;
            turn_rate = deg2rad(targetConfig.evasive_turn_rate_deg);

            if (t_now - targetState.last_turn_flip_time) >= flip_interval
                targetState.turn_sign = -targetState.turn_sign;
                targetState.last_turn_flip_time = t_now;
            end

            T_heading = T_heading + targetState.turn_sign * turn_rate * dt;

        case 3
            % Random: 일정 시간마다 랜덤 turn rate 갱신
            update_interval = targetConfig.random_update_interval;
            max_turn_rate = deg2rad(targetConfig.random_turn_rate_max_deg);

            if (t_now - targetState.last_random_update_time) >= update_interval
                targetState.random_turn_rate = (2*rand - 1) * max_turn_rate;
                targetState.last_random_update_time = t_now;
            end

            T_heading = T_heading + targetState.random_turn_rate * dt;

        case 4
            % Reactive: chaser가 가까우면 멀어지는 방향으로 turn
            reactive_range = targetConfig.reactive_range;
            reactive_turn_rate = deg2rad(targetConfig.reactive_turn_rate_deg);

            rel = T_pos - chaser_pos;
            dist = norm(rel);

            if dist < reactive_range
                desired_heading = atan2(rel(2), rel(1));   % away from chaser
                heading_error = wrapAngle(desired_heading - T_heading);

                max_step = reactive_turn_rate * dt;
                heading_step = max(min(heading_error, max_step), -max_step);

                T_heading = T_heading + heading_step;
            end

        case 5
            rel = T_pos - chaser_pos;
            dist = norm(rel);

            % 1) panic 진입 / 해제 조건
            if ~targetState.panic_active
                if dist < targetConfig.panic_range
                    targetState.panic_active = true;
                    targetState.panic_start_time = t_now;
                end
            else
                hold_done  = (t_now - targetState.panic_start_time) >= targetConfig.panic_hold_time;
                far_enough = dist > targetConfig.recover_range;

                if hold_done && far_enough
                    targetState.panic_active = false;
                end
            end

            % 2) panic 상태면 "왼쪽 bias가 섞인 away 방향" 하나만 사용
            if targetState.panic_active
                away_dir = rel / norm(rel);   % chaser -> target 방향 = away
                left_dir = [-1; 0];           % 화면 기준 왼쪽 bias

                alpha = 1.0;                  % away 성분
                beta  = 0.6;                  % left bias 성분

                biased_dir = alpha * away_dir + beta * left_dir;
                biased_dir = biased_dir / norm(biased_dir);

                desired_heading = atan2(biased_dir(2), biased_dir(1));

                heading_error = wrapAngle(desired_heading - T_heading);
                max_step = deg2rad(targetConfig.panic_turn_rate_deg) * dt;
                heading_step = max(min(heading_error, max_step), -max_step);

                T_heading = T_heading + heading_step;
            end
    end

    T_vel = T_speed * [cos(T_heading); sin(T_heading)];
    T_pos = T_pos + T_vel * dt;
end

function ang = wrapAngle(ang)
    ang = atan2(sin(ang), cos(ang));
end