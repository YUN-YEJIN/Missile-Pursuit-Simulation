clc; clear; close all;

%% ================================
%  1. Simulation setting
% ================================
dt = 0.01;          % time step [s]
T_end = 600;         % total simulation time [s]
time = 0:dt:T_end;

hit_radius = 10;    % interception condition [m]

%% ================================
%  2. Interceptor initial condition
% ================================
M_pos = [0; 0];             % interceptor initial position [x; y]
M_speed = 300;              % interceptor speed [m/s]
M_heading_deg = 20;         % interceptor initial heading [deg]
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
%  4. Guidance parameters
% ================================
N = 3;              % navigation constant
a_max = 80;         % max lateral acceleration [m/s^2]

%pid로 튜닝해보자!

%% ================================
%  5. Data save
% ================================
M_hist = zeros(2, length(time));
T_hist = zeros(2, length(time));
R_hist = zeros(1, length(time));
a_hist = zeros(1, length(time));

%% ================================
%  6. Main simulation loop
% ================================
T_vel_prev = T_vel;
figure;

h_M = plot(NaN, NaN, 'b', 'LineWidth', 2); hold on;
h_T = plot(NaN, NaN, 'r', 'LineWidth', 2);
h_M_start = plot(M_pos(1), M_pos(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
h_T_start = plot(T_pos(1), T_pos(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
h_M_cur = plot(NaN, NaN, 'b^', 'MarkerSize', 8, 'LineWidth', 2);
h_T_cur = plot(NaN, NaN, 'r^', 'MarkerSize', 8, 'LineWidth', 2);
grid on; axis equal;
xlabel('X Position [m]');
ylabel('Y Position [m]');
legend('Interceptor', 'Target', 'Interceptor start', 'Target start', ...
       'Interceptor cur', 'Target cur');
title('APN Guidance Simulation (Real-time)');

update_interval = round(0.1 / dt);

for k = 1:length(time)

    % Save current position
    M_hist(:, k) = M_pos;
    T_hist(:, k) = T_pos;

    % Relative position and velocity
    r = T_pos - M_pos;
    v_rel = T_vel - M_vel;

    % Distance between interceptor and target
    R = norm(r);
    R_hist(k) = R;

    % Check interception
    if R < hit_radius
        fprintf("Intercept success at t = %.2f sec\n", time(k));

        M_hist = M_hist(:, 1:k);
        T_hist = T_hist(:, 1:k);
        R_hist = R_hist(1:k);
        a_hist = a_hist(1:k);
        time = time(1:k);
        break;
    end
    % 실시간 갱신 (루프 안)
    if mod(k, update_interval) == 0 || k == 1
        set(h_M, 'XData', M_hist(1, 1:k), 'YData', M_hist(2, 1:k));
        set(h_T, 'XData', T_hist(1, 1:k), 'YData', T_hist(2, 1:k));
        set(h_M_cur, 'XData', M_pos(1), 'YData', M_pos(2));
        set(h_T_cur, 'XData', T_pos(1), 'YData', T_pos(2));
        drawnow;
    end

    % 여기부터는 비례항법공식

    % LOS rate calculation
    lambda_dot = (r(1)*v_rel(2) - r(2)*v_rel(1)) / R^2;

    % Closing velocity
    Vc = -dot(r, v_rel) / R;
    
    % Augmented Proportional Navigation Guidance
    if k == 1
        a_T_est = [0; 0];
    else
        a_T_est = (T_vel - T_vel_prev) / dt;  % 표적 가속도 추정
    end
    T_vel_prev = T_vel;
    los_dir = r / R;
    a_T_perp = a_T_est - dot(a_T_est, los_dir) * los_dir;
    
    % Lateral acceleration direction
    M_speed_now = norm(M_vel);
    normal_dir = [-M_vel(2); M_vel(1)] / M_speed_now;

    a_cmd = N * Vc * lambda_dot + 0.5 * N *dot(a_T_perp, normal_dir);

    % Acceleration limit
    a_cmd = max(min(a_cmd, a_max), -a_max);
    a_hist(k) = a_cmd;

    % Interceptor acceleration
    a_M = a_cmd * normal_dir;

    % Update interceptor velocity and position
    M_vel = M_vel + a_M * dt;

    % Constant speed assumption
    M_vel = M_speed * M_vel / norm(M_vel);

    M_pos = M_pos + M_vel * dt;

    % Update target position
    % Target moves straight with constant velocity
    % Target maneuver: zig-zag evasive motion

    a_T_mag = 20 * sin(2*pi*0.3*time(k));
    T_speed_current = norm(T_vel);
    normal_T = [-T_vel(2); T_vel(1)] / T_speed_current;
    a_T = a_T_mag * normal_T;


    T_vel = T_vel + a_T * dt;
    T_vel = T_speed_current * T_vel / norm(T_vel);
    T_pos = T_pos + T_vel * dt;

end
%% ================================
%  7. Plot trajectory
% ================================

set(h_M_start, 'XData', M_hist(1,1), 'YData', M_hist(2,1));
set(h_T_start, 'XData', T_hist(1,1), 'YData', T_hist(2,1));
set(h_M_cur, 'XData', M_hist(1,end), 'YData', M_hist(2,end));
set(h_T_cur, 'XData', T_hist(1,end), 'YData', T_hist(2,end));
drawnow;

%% ================================
%  8. Plot range
% ================================
figure;
plot(time, R_hist, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Range [m]');
title('Distance between Interceptor and Target');

%% ================================
%  9. Plot acceleration command
% ================================
figure;
plot(time, a_hist, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Acceleration Command [m/s^2]');
title('PN Acceleration Command');