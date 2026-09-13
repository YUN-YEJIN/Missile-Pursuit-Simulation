clc; clear; close all;

%% ================================
%  1. Simulation setting
% ================================
dt = 0.01;          % time step [s]
T_end = 60;         % total simulation time [s]
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

    % 여기부터는 비례항법공식

    % LOS rate calculation
    lambda_dot = (r(1)*v_rel(2) - r(2)*v_rel(1)) / R^2;

    % Closing velocity
    Vc = -dot(r, v_rel) / R;

    % Proportional Navigation Guidance
    a_cmd = N * Vc * lambda_dot;

    % Acceleration limit
    a_cmd = max(min(a_cmd, a_max), -a_max);
    a_hist(k) = a_cmd;

    % Lateral acceleration direction
    M_speed_now = norm(M_vel);
    normal_dir = [-M_vel(2); M_vel(1)] / M_speed_now;

    % Interceptor acceleration
    a_M = a_cmd * normal_dir;

    % Update interceptor velocity and position
    M_vel = M_vel + a_M * dt;

    % Constant speed assumption
    M_vel = M_speed * M_vel / norm(M_vel);

    M_pos = M_pos + M_vel * dt;

    % Update target position
    % Target moves straight with constant velocity
    T_pos = T_pos + T_vel * dt;

end

%% ================================
%  7. Plot trajectory
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
legend('Interceptor trajectory', 'Target trajectory', ...
       'Interceptor start', 'Target start', ...
       'Interceptor end', 'Target end');

title('Pure Proportional Navigation Guidance Simulation');

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