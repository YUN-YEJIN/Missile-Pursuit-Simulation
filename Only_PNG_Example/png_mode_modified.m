clc; clear; close all;

%% ================================
%  1. Simulation setting
% ================================
dt = 0.01;          
T_end = 60;         
time = 0:dt:T_end;

hit_radius = 10;    

%% ================================
%  2. Chaser initial condition
% ================================
M_pos = [0; 0];
M_speed = 300;
M_heading_deg = 20;
M_heading = deg2rad(M_heading_deg);

M_vel = M_speed * [cos(M_heading); sin(M_heading)];

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
disp("0 : Random switching among 1~4");

target_mode_input = input("Select target mode = ");

if target_mode_input == 0
    use_random_switching = true;
    target_mode = randi(4);
else
    use_random_switching = false;
    target_mode = target_mode_input;
end

% mode switching interval when random switching is on
mode_switch_interval = 3.0;   % [s]

% target motion state/config
targetState.turn_sign = 1;
targetState.last_turn_flip_time = 0;
targetState.last_random_update_time = 0;
targetState.random_turn_rate = 0;

targetConfig.evasive_turn_rate_deg = 18;     % weaving turn rate
targetConfig.evasive_flip_interval = 1.2;    % direction change interval [s]

targetConfig.random_turn_rate_max_deg = 30;  % random turn rate bound
targetConfig.random_update_interval = 1.0;   % how often random turn rate changes [s]

targetConfig.reactive_turn_rate_deg = 28;    % reactive turning speed
targetConfig.reactive_range = 700;           % react only when chaser is close [m]

%% ================================
%  5. Guidance parameters
% ================================
N = 3;
a_max = 80;

%% ================================
%  6. Data save
% ================================
M_hist = zeros(2, length(time));
T_hist = zeros(2, length(time));
R_hist = zeros(1, length(time));
a_hist = zeros(1, length(time));
mode_hist = zeros(1, length(time));

success = false;

%% ================================
%  7. Main simulation loop
% ================================
for k = 1:length(time)

    t_now = time(k);

    % random mode switching
    if use_random_switching
        if k == 1 || mod(t_now, mode_switch_interval) < dt
            target_mode = randi(4);
        end
    end

    % Save current position
    M_hist(:, k) = M_pos;
    T_hist(:, k) = T_pos;
    mode_hist(k) = target_mode;

    % Relative position and velocity
    r = T_pos - M_pos;
    v_rel = T_vel - M_vel;

    % Distance
    R = norm(r);
    R_hist(k) = R;

    % Check encounter
    if R < hit_radius
        fprintf("Encounter success at t = %.2f sec\n", t_now);

        M_hist = M_hist(:, 1:k);
        T_hist = T_hist(:, 1:k);
        R_hist = R_hist(1:k);
        a_hist = a_hist(1:k);
        mode_hist = mode_hist(1:k);
        time = time(1:k);

        success = true;
        break;
    end

    % LOS rate
    lambda_dot = (r(1)*v_rel(2) - r(2)*v_rel(1)) / R^2;

    % Closing velocity
    Vc = -dot(r, v_rel) / R;

    % Proportional Navigation
    a_cmd = N * Vc * lambda_dot;

    % Acceleration limit
    a_cmd = max(min(a_cmd, a_max), -a_max);
    a_hist(k) = a_cmd;

    % Lateral acceleration direction
    M_speed_now = norm(M_vel);
    normal_dir = [-M_vel(2); M_vel(1)] / M_speed_now;

    % Chaser acceleration
    a_M = a_cmd * normal_dir;

    % Update chaser velocity and position
    M_vel = M_vel + a_M * dt;
    M_vel = M_speed * M_vel / norm(M_vel);   % constant speed
    M_pos = M_pos + M_vel * dt;

    % Update target motion according to selected mode
    [T_pos, T_vel, T_heading, targetState] = updateTargetMotion( ...
        T_pos, T_speed, T_heading, dt, t_now, target_mode, ...
        targetState, targetConfig, M_pos);
end

if ~success
    fprintf("Simulation ended without encounter.\n");
    fprintf("Final range = %.2f m\n", R_hist(find(R_hist>0,1,'last')));
end

%% ================================
%  8. Plot trajectory
% ================================
figure;
plot(M_hist(1,:), M_hist(2,:), 'b', 'LineWidth', 2); hold on;
plot(T_hist(1,:), T_hist(2,:), 'r', 'LineWidth', 2);

plot(M_hist(1,1), M_hist(2,1), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
plot(T_hist(1,1), T_hist(2,1), 'ro', 'MarkerSize', 8, 'LineWidth', 2);

plot(M_hist(1,end), M_hist(2,end), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
plot(T_hist(1,end), T_hist(2,end), 'rx', 'MarkerSize', 10, 'LineWidth', 2);

grid on;
axis equal;
xlabel('X Position [m]');
ylabel('Y Position [m]');
legend('Chaser trajectory', 'Target trajectory', ...
       'Chaser start', 'Target start', ...
       'Chaser end', 'Target end');
title('2D Pursuit-Evasion Simulation');

%% ================================
%  9. Plot range
% ================================
figure;
plot(time, R_hist, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Range [m]');
title('Distance between Chaser and Target');

%% ================================
% 10. Plot acceleration command
% ================================
figure;
plot(time, a_hist, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Acceleration Command [m/s^2]');
title('PN-like Acceleration Command');

%% ================================
% 11. Plot target mode history
% ================================
figure;
stairs(time, mode_hist, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Target Mode');
yticks([1 2 3 4]);
ylim([0.5 4.5]);
title('Target Behavior Mode History');

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
            % Evasive: 좌우로 weave
            flip_interval = targetConfig.evasive_flip_interval;
            turn_rate = deg2rad(targetConfig.evasive_turn_rate_deg);

            if (t_now - targetState.last_turn_flip_time) >= flip_interval
                targetState.turn_sign = -targetState.turn_sign;
                targetState.last_turn_flip_time = t_now;
            end

            T_heading = T_heading + targetState.turn_sign * turn_rate * dt;

        case 3
            % Random: 일정 시간마다 랜덤 turn rate 재설정
            update_interval = targetConfig.random_update_interval;
            max_turn_rate = deg2rad(targetConfig.random_turn_rate_max_deg);

            if (t_now - targetState.last_random_update_time) >= update_interval
                targetState.random_turn_rate = (2*rand - 1) * max_turn_rate;
                targetState.last_random_update_time = t_now;
            end

            T_heading = T_heading + targetState.random_turn_rate * dt;

        case 4
            % Reactive: chaser가 가까워지면 반대 방향으로 heading 수정
            reactive_range = targetConfig.reactive_range;
            reactive_turn_rate = deg2rad(targetConfig.reactive_turn_rate_deg);

            rel = T_pos - chaser_pos;
            dist = norm(rel);

            if dist < reactive_range
                desired_heading = atan2(rel(2), rel(1));  % chaser 반대 방향
                heading_error = wrapAngle(desired_heading - T_heading);

                max_step = reactive_turn_rate * dt;
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