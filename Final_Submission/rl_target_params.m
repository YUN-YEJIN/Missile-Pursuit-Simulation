%% rl_target_params.m
% Reinforcement-learning target settings

% Target and agent update interval
target_dt = 0.01;          % [s], 100 Hz

% 근거리 RL 회피 진입 거리
R_RL_enter = 2200.0;   % [m]

% Episode termination conditions
R_capture = 100.0;         % [m]
max_episode_time = 6.0;    % [s]

% Observation normalization
R_scale = 5000.0;          % [m]
V_scale = 1200.0;          % [m/s]

% Target maneuver limit
% Action = ±1 means target performs ±target_max_g maneuver.
target_max_g = 12.0;
target_max_turn_rate = target_max_g*g/v_tgt;  % [rad/s]

% Initial geometric range
missile_pos_initial = [x_ini; -h_ini];
range_init = norm(pos_tgt(:) - missile_pos_initial);

% RL 근거리 회피 구간
R_RL_enter = 2200.0;   % RL 진입 거리 [m]
R_RL_exit  = 2600.0;   % RL 종료 거리 [m]

% 터미널 회피 진입 거리
R_terminal = 1200.0;   % [m]