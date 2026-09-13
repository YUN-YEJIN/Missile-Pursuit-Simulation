%% run_trained_agent_random.m
clear; clc; close all;

% 매번 다른 랜덤 초기조건을 원하면 shuffle
rng("shuffle");

% 1. 기본 파라미터 로드
aero_guid_dat;

% 2. 모델 열기
mdl = "aero_guidance_train";
open_system(mdl);

% 3. 학습된 agent 파일 경로
agentFile = "C:\Users\sunny\OneDrive\바탕 화면\RL + destination\saved_agents\run_20260806_084500\best_candidates\Agent312.mat";

% 4. agent 불러오기
S = load(agentFile);

disp("Loaded variables:");
disp(fieldnames(S));

if isfield(S, "finalAgent")
    agent = S.finalAgent;
elseif isfield(S, "saved_agent")
    agent = S.saved_agent;
elseif isfield(S, "agent")
    agent = S.agent;
else
    error("이 mat 파일 안에서 agent 변수를 찾지 못했습니다.");
end

% Simulink RL Agent 블록이 볼 수 있게 base workspace에 넣기
assignin("base", "agent", agent);

% 5. RL Agent 블록에 agent 연결
agentBlk = mdl + "/Target Position/RL Agent";
set_param(agentBlk, "Agent", "agent");

% 6. 시뮬레이션 시간 설정
set_param(mdl, "StopTime", num2str(max_episode_time));

% 7. Animation 켜기
try
    set_param(mdl + "/3DoF Animation", "u8", "on");
catch ME
    warning("3DoF Animation을 켜지 못했습니다.\n%s", ME.message);
end

% % 8. 랜덤 초기조건을 SimulationInput에 적용
% simIn = Simulink.SimulationInput(mdl);
% simIn = applyRandomInitialCondition(simIn);
% 
% % 9. 실행
% simOut = sim(simIn);
N = 10;

for i = 1:N
    fprintf("\n===== Random test %d/%d =====\n", i, N);

    simIn = Simulink.SimulationInput(mdl);
    simIn = applyRandomInitialCondition(simIn);

    simOut = sim(simIn);
end


%% ===============================================================
% Local function: random initial condition
% ===============================================================
function in = applyRandomInitialCondition(in)

    % X-Z bounds
    x_min = 0.0;
    x_max = 5000.0;

    z_min = -5050.0;
    z_max = -3050.0;

    % 초기 거리 조건
    min_sep = 4500.0;
    max_sep = 5400.0;

    % 미사일 시야/PN에 너무 불리하지 않게 LOS 제한
    max_los_angle = 10.0 * pi / 180.0;

    maxTry = 10000;
    valid = false;

    for k = 1:maxTry

        % Missile: left side, lower altitude
        missile_x0 = 200.0 + 600.0 * rand;       % 200 ~ 800
        missile_z0 = -4250.0 + 700.0 * rand;     % -4250 ~ -3550
        
        % Target: right side, above missile
        target_x0 = 4200.0 + 700.0 * rand;       % 4200 ~ 4900
        
        z_offset = 250.0 + 350.0 * rand;         % 250 ~ 600 m vertical gap
        
        % 주의:
        % z = -altitude 이므로, target이 위에 있으려면 target_z0가 더 작아야 함
        target_z0 = missile_z0 - z_offset;

        dx = target_x0 - missile_x0;
        dz = target_z0 - missile_z0;

        init_sep = sqrt(dx^2 + dz^2);
        los_angle = atan2(dz, dx);

        % Goal: missile 근처이되 정면 충돌선에서 살짝 벗어나게 배치
        goal_x = min(missile_x0 + 300.0, x_max);

        if rand < 0.5
            goal_z = missile_z0 + 600.0;
        else
            goal_z = missile_z0 - 600.0;
        end

        goal_z = min(max(goal_z, z_min), z_max);

        goal_inside = ...
            goal_x >= x_min && goal_x <= x_max && ...
            goal_z >= z_min && goal_z <= z_max;
        target_inside = ...
            target_z0 >= z_min && target_z0 <= z_max;

        if dx > 0 && ...
           init_sep >= min_sep && ...
           init_sep <= max_sep && ...
           abs(los_angle) <= max_los_angle && ...
           goal_inside && ...
           target_inside
        
            valid = true;
            break;
        end
    end

    if ~valid
        missile_x0 = 500.0;
        missile_z0 = -3800.0;

        target_x0 = 4700.0;
        target_z0 = -4300.0;

        goal_x = 800.0;
        goal_z = -3600.0;

        dx = target_x0 - missile_x0;
        dz = target_z0 - missile_z0;

        init_sep = sqrt(dx^2 + dz^2);
        los_angle = atan2(dz, dx);
    end

    % Missile heading
    theta_ini_reset = 0.0;

    % Target heading toward goal
    theta_tgt_reset = atan2( ...
        goal_z - target_z0, ...
        goal_x - target_x0);

    % Model convention:
    % missile x = x_ini
    % missile z = -h_ini
    x_ini_reset = missile_x0;
    h_ini_reset = -missile_z0;

    target_init_pos_reset = [target_x0;
                             target_z0];

    pos_tgt_reset = target_init_pos_reset.';

    Goal_pos_reset = [goal_x;
                      goal_z];

    % Set variables for this simulation
    in = setVariable(in, "x_ini", x_ini_reset);
    in = setVariable(in, "h_ini", h_ini_reset);
    in = setVariable(in, "theta_ini", theta_ini_reset);

    in = setVariable(in, "target_init_pos", target_init_pos_reset);
    in = setVariable(in, "pos_tgt", pos_tgt_reset);
    in = setVariable(in, "theta_tgt", theta_tgt_reset);

    in = setVariable(in, "Goal_pos", Goal_pos_reset);

    fprintf('Random IC applied:\n');
    
    fprintf('Missile [%.1f, %.1f], Target [%.1f, %.1f], Goal [%.1f, %.1f]\n', ...
        missile_x0, missile_z0, ...
        target_x0, target_z0, ...
        goal_x, goal_z);
    
    fprintf('Separation %.1f m, LOS %.1f deg, theta_t %.1f deg\n', ...
        init_sep, ...
        los_angle * 180/pi, ...
        theta_tgt_reset * 180/pi);

end