%% train_target_evasion_sac.m
clear; clc; close all;


% 1. 파라미터 로드
aero_guid_dat;

mdl = "aero_guidance_train";
open_system(mdl);

% 2. Agent 블록 경로 지정
agentBlk = mdl + "/Target Position/RL Agent";

%% Disable original plotting and original fuze stop during RL training
% RL isDone must terminate each episode.
set_param(mdl, "StopFcn", "");
set_param(mdl, "StopTime", num2str(max_episode_time));

try
    set_param(mdl + "/Guidance/Fuze/Stop Simulation", "Commented", "on");
catch ME
    warning("Could not comment out the original Stop Simulation block.\n%s", ME.message);
end

% Animation is unnecessary during training.
try
    set_param(mdl + "/3DoF Animation", "u8", "off");
catch
    % Mask parameters can differ by release; training can continue.
end

%% 3. Spec 설정
obsInfo = rlNumericSpec([9 1]);
obsInfo.Name = "target_observation";
actInfo = rlNumericSpec([1 1], 'LowerLimit', -1, 'UpperLimit', 1);
actInfo.Name = "target_turn_action";

% 4. SAC Agent 생성
rng(0);
agentOpts = rlSACAgentOptions;
agentOpts.SampleTime = target_dt;
agentOpts.DiscountFactor = 0.995;
agentOpts.MiniBatchSize = 256;
agentOpts.ExperienceBufferLength = 1e6;
agent = rlSACAgent(obsInfo, actInfo, agentOpts);

% 5. Simulink RL 환경 구성
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
env.UseFastRestart = "off";
env.ResetFcn = @(in) localResetFcn(in);

% 6. 환경 검증
disp('Validating RL Environment...');
validateEnvironment(env);

% 7. 저장 폴더 생성
run_id = "run_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss"));

save_dir = fullfile("./saved_agents", run_id);
final_dir = fullfile(save_dir, "final");
best_dir  = fullfile(save_dir, "best_candidates");

if ~exist(final_dir, 'dir')
    mkdir(final_dir);
end

if ~exist(best_dir, 'dir')
    mkdir(best_dir);
end

% 8. 학습 옵션 설정 (하나로 통합)
maxEpisodeTime = max_episode_time;
maxSteps = ceil(maxEpisodeTime / target_dt);

trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 600, ...
    'MaxStepsPerEpisode', maxSteps, ...
    'ScoreAveragingWindowLength', 20, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 180.0, ...
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', -500.0, ...
    'SaveAgentDirectory', best_dir, ...
    'Plots', 'training-progress', ...
    'Verbose', true);

% 9. 학습 시작 (한 번만)
disp('Starting SAC Training...');
trainingStats = train(agent, env, trainOpts);

% 10. 학습 완료 후 저장
% 10. 학습 완료 후 final 모델 저장
finalAgent = agent;

final_file = fullfile(final_dir, "Final_Target_Evasion_SAC_Agent.mat");

save(final_file, ...
    'finalAgent', ...
    'agent', ...
    'trainingStats', ...
    'trainOpts', ...
    'obsInfo', ...
    'actInfo', ...
    'maxEpisodeTime', ...
    'maxSteps');

disp("Final 모델 저장 완료:");
disp(final_file);

disp("Best 후보 모델 저장 폴더:");
disp(best_dir);

% 매 에피소드 초기화 구동
function in = localResetFcn(in)

    % ==============================================================
    % Random initial condition for missile-target-goal engagement
    % Axes limit: [0 5000 -2000 2000 -5050 -3050]
    % Current model uses X-Z plane.
    % ==============================================================

    % X-Z bounds
    x_min = 0.0;
    x_max = 5000.0;

    z_min = -5050.0;
    z_max = -3050.0;

    % Initial separation condition
    min_sep = 4500.0;
    max_sep = 5400.0;

    % Initial LOS angle limit for PN-friendly engagement
    max_los_angle = 10.0 * pi / 180.0;

    % Goal should be near missile initial position
    goal_min_offset = 200.0;   % 너무 missile과 같은 점 방지
    goal_max_offset = 700.0;   % missile 근처 목적지 반경

    maxTry = 10000;
    valid = false;

    for k = 1:maxTry

        % ----------------------------------------------------------
        % 1) Missile: left side
        % ----------------------------------------------------------
        missile_x0 = 200.0 + 600.0 * rand;       % 200 ~ 800

        missile_z0 = -4850.0 + 1000.0 * rand;    % -4850 ~ -3850

        z_offset = 200.0 + 400.0 * rand;         % 200 ~ 600 m 위쪽

        target_z0 = missile_z0 + z_offset;
        % missile_z0 = -4850.0 + 1300.0 * rand;    % -4850 ~ -3550

        % ----------------------------------------------------------
        % 2) Target: right side
        % ----------------------------------------------------------
        target_x0 = 4200.0 + 700.0 * rand;       % 4200 ~ 4900
        % target_z0 = -4850.0 + 1300.0 * rand;     % -4850 ~ -3550

        dx = target_x0 - missile_x0;
        dz = target_z0 - missile_z0;

        init_sep = sqrt(dx^2 + dz^2);
        los_angle = atan2(dz, dx);

        % ----------------------------------------------------------
        % 3) Goal: near missile initial position
        % ----------------------------------------------------------
        goal_r = goal_min_offset + ...
            (goal_max_offset - goal_min_offset) * rand;

        goal_ang = 2*pi*rand;

        goal_x = missile_x0 + goal_r*cos(goal_ang);
        goal_z = missile_z0 + goal_r*sin(goal_ang);

        % Goal must also be inside axes range
        goal_inside = ...
            goal_x >= x_min && goal_x <= x_max && ...
            goal_z >= z_min && goal_z <= z_max;

        % ----------------------------------------------------------
        % 4) Valid engagement condition
        % ----------------------------------------------------------
        if dx > 0 && ...
           init_sep >= min_sep && ...
           init_sep <= max_sep && ...
           abs(los_angle) <= max_los_angle && ...
           goal_inside

            valid = true;
            break;
        end
    end

    % ==============================================================
    % Fallback if random sampling fails
    % ==============================================================

    if ~valid
        missile_x0 = 500.0;
        missile_z0 = -4200.0;

        target_x0 = 4700.0;
        target_z0 = -4200.0;

        goal_x = 700.0;
        goal_z = -4200.0;

        dx = target_x0 - missile_x0;
        dz = target_z0 - missile_z0;

        init_sep = sqrt(dx^2 + dz^2);
        los_angle = atan2(dz, dx);
    end

    % ==============================================================
    % Initial heading setting
    % ==============================================================

    % Option A: 기존 PN 예제와 비슷하게 missile은 오른쪽을 봄
    theta_ini_reset = 0.0;

    % Option B: missile이 target을 정확히 바라보게 하고 싶으면 위 줄 대신 아래 사용
    % theta_ini_reset = atan2(target_z0 - missile_z0, target_x0 - missile_x0);

    % Target starts by looking toward its goal
    theta_tgt_reset = atan2( ...
        goal_z - target_z0, ...
        goal_x - target_x0);

    % ==============================================================
    % Convert missile position to model variables
    %
    % Model convention:
    %   missile x = x_ini
    %   missile z = -h_ini
    % ==============================================================

    x_ini_reset = missile_x0;
    h_ini_reset = -missile_z0;

    target_init_pos_reset = [target_x0;
                             target_z0];

    pos_tgt_reset = target_init_pos_reset.';

    Goal_pos_reset = [goal_x;
                      goal_z];

    % ==============================================================
    % Set variables for this episode
    % ==============================================================

    in = setVariable(in, 'x_ini', x_ini_reset);
    in = setVariable(in, 'h_ini', h_ini_reset);
    in = setVariable(in, 'theta_ini', theta_ini_reset);

    in = setVariable(in, 'target_init_pos', target_init_pos_reset);
    in = setVariable(in, 'pos_tgt', pos_tgt_reset);
    in = setVariable(in, 'theta_tgt', theta_tgt_reset);

    in = setVariable(in, 'Goal_pos', Goal_pos_reset);

    % % Debug print
    % fprintf(['Missile [%.1f, %.1f], Target [%.1f, %.1f], ', ...
    %          'Goal [%.1f, %.1f], Sep %.1f, LOS %.1f deg, ', ...
    %          'theta_m %.1f deg, theta_t %.1f deg\n'], ...
    %     missile_x0, missile_z0, ...
    %     target_x0, target_z0, ...
    %     goal_x, goal_z, ...
    %     init_sep, ...
    %     los_angle * 180/pi, ...
    %     theta_ini_reset * 180/pi, ...
    %     theta_tgt_reset * 180/pi);

end