function varargout = targetModeFunction(action, varargin)
% targetModeFunction
% -------------------------------------------------------------------------
% Target behavior mode 관련 기능을 한 파일에 모은 함수.
%
% 사용법
%   [target_mode, use_random_switching, mode_switch_steps, targetState, targetConfig]
%       = targetModeFunction("setup", dt)
%
%   target_mode = targetModeFunction("switch", k, use_random_switching,
%       target_mode, mode_switch_steps)
%
%   [T_pos, T_vel, T_heading, targetState] = targetModeFunction("update",
%       T_pos, T_speed, T_heading, dt, t_now, target_mode,
%       targetState, targetConfig, chaser_pos)
% -------------------------------------------------------------------------

    action = char(action);

    switch lower(action)

        case 'setup'
            dt = varargin{1};

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

            % Target state
            targetState.turn_sign = 1;
            targetState.last_turn_flip_time = 0;
            targetState.last_random_update_time = 0;
            targetState.random_turn_rate = 0;

            % Panic state machine
            targetState.panic_active = false;
            targetState.panic_start_time = -inf;

            % Target config
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

            varargout = {target_mode, use_random_switching, mode_switch_steps, ...
                         targetState, targetConfig};

        case 'switch'
            k = varargin{1};
            use_random_switching = varargin{2};
            target_mode = varargin{3};
            mode_switch_steps = varargin{4};

            if use_random_switching
                if k == 1 || mod(k-1, mode_switch_steps) == 0
                    target_mode = randi(5);
                end
            end

            varargout = {target_mode};

        case 'update'
            T_pos = varargin{1};
            T_speed = varargin{2};
            T_heading = varargin{3};
            dt = varargin{4};
            t_now = varargin{5};
            target_mode = varargin{6};
            targetState = varargin{7};
            targetConfig = varargin{8};
            chaser_pos = varargin{9};

            [T_pos, T_vel, T_heading, targetState] = updateTargetMotion( ...
                T_pos, T_speed, T_heading, dt, t_now, target_mode, ...
                targetState, targetConfig, chaser_pos);

            varargout = {T_pos, T_vel, T_heading, targetState};

        otherwise
            error("Unknown targetModeFunction action: %s", action);
    end
end

%% ================================
%  Local target motion function
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
            % Reactive: chaser가 가까우면 chaser에서 멀어지는 방향으로 turn
            reactive_range = targetConfig.reactive_range;
            reactive_turn_rate = deg2rad(targetConfig.reactive_turn_rate_deg);

            rel = T_pos - chaser_pos;
            dist = norm(rel);

            if dist < reactive_range
                desired_heading = atan2(rel(2), rel(1));   % away from chaser
                heading_error = wrapAngleLocal(desired_heading - T_heading);

                max_step = reactive_turn_rate * dt;
                heading_step = max(min(heading_error, max_step), -max_step);

                T_heading = T_heading + heading_step;
            end

        case 5
            % Panic Escape: 일정 거리 안으로 들어오면 급격하게 회피
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

            % 2) panic 상태면 "away 방향 + 왼쪽 bias" 방향으로 회피
            if targetState.panic_active
                if dist < eps
                    away_dir = [cos(T_heading); sin(T_heading)];
                else
                    away_dir = rel / dist;   % chaser -> target 방향 = away
                end

                left_dir = [-1; 0];          % 화면 기준 왼쪽 bias

                alpha = 1.0;                 % away 성분
                beta  = 0.6;                 % left bias 성분

                biased_dir = alpha * away_dir + beta * left_dir;
                biased_dir = biased_dir / norm(biased_dir);

                desired_heading = atan2(biased_dir(2), biased_dir(1));

                heading_error = wrapAngleLocal(desired_heading - T_heading);
                max_step = deg2rad(targetConfig.panic_turn_rate_deg) * dt;
                heading_step = max(min(heading_error, max_step), -max_step);

                T_heading = T_heading + heading_step;
            end

        otherwise
            error("Unknown target mode: %d", target_mode);
    end

    T_vel = T_speed * [cos(T_heading); sin(T_heading)];
    T_pos = T_pos + T_vel * dt;
end

function ang = wrapAngleLocal(ang)
    ang = atan2(sin(ang), cos(ang));
end
