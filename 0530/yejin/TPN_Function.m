function varargout = TPN_Function(action, varargin)
% targetModeFunction
% -------------------------------------------------------------------------
% Target behavior mode 관련 기능을 한 파일에 모은 함수.
% -------------------------------------------------------------------------

    action = char(action);

    switch lower(action)

        case 'setup'
            dt = varargin{1};

            disp("=== Target Mode Selection ===");
            disp("1 : Straight");
            disp("2 : Panic Evasion");
            disp("3 : LOS Perpendicular Evasion");
            disp("4 : Optimal Evasion (Collision Triangle)");

            target_mode = input("Select target mode = ");

            % Target state 초기화
            targetState.panic_active = false;
            targetState.panic_start_time = -inf;

            targetState.panic_perp_dir = [0; 1];  
            targetState.last_random_update_time = 0;
            targetState.random_turn_rate = 0;
            targetState.evasion_offset = 0;   
            targetState.evasion_sign = 1;         % Mode 3 좌/우 회피 방향 부호
            targetState.just_updated_turn = false; 

            % Target config
            % Mode 2: Panic Evasion 
            targetConfig.random_turn_rate_max_deg = 120;  % [deg/s] 최대 선회율
            targetConfig.random_update_interval   = 1.0;  % [s] turn rate 갱신 주기
            targetConfig.panic_range              = 1000; % [m] 반응 시작 거리

            % Mode 3: LOS Perpendicular
            targetConfig.los_perp_turn_rate_deg = 180;     % [deg/s] 선회율 제한 (180도로 상향)
            targetConfig.los_perp_range         = 1000;    % [m] 반응 시작 거리
 
            % Mode 4: Optimal Evasion (Collision Triangle)
            targetConfig.collision_turn_rate_deg = 180;    % [deg/s] 선회율 제한 (180도로 상향)
            targetConfig.collision_range         = 1000;  % [m]

            % 공통: 회피 후 목적지 복귀 blending 파라미터
            targetConfig.evasion_blend_out_time   = 3.0;  % [s] 위협 사라진 후 복귀 시간
            targetConfig.max_evasion_offset_deg   = 90;   % [deg] 기본 최대 허용 회피 오프셋

            varargout = {target_mode, targetState, targetConfig};

        case 'update'
            T_pos        = varargin{1};
            T_speed      = varargin{2};
            T_heading    = varargin{3};
            base_heading = varargin{4};   
            dt           = varargin{5};
            t_now        = varargin{6};
            target_mode  = varargin{7};
            targetState  = varargin{8};
            targetConfig = varargin{9};
            C_pos        = varargin{10};
            C_vel        = varargin{11};
            C_speed      = varargin{12};

            [T_pos, T_vel, T_heading, targetState] = updateTargetMotion( ...
                T_pos, T_speed, T_heading, base_heading, dt, t_now, target_mode, ...
                targetState, targetConfig, C_pos, C_vel, C_speed);

            varargout = {T_pos, T_vel, T_heading, targetState};

        otherwise
            error("Unknown targetModeFunction action: %s", action);
    end
end

%% ================================
%  Local target motion function
% ================================
function [T_pos, T_vel, T_heading, targetState] = updateTargetMotion( ...
    T_pos, T_speed, T_heading, base_heading, dt, t_now, target_mode, ...
    targetState, targetConfig, C_pos, C_vel, C_speed)

    switch target_mode

        case 1
            % Straight: 회피 없음, 오프셋 0 유지
            targetState.evasion_offset = 0;
            targetState.just_updated_turn = false;

        case 2
            % Panic Evasion: 1000m 이내일 때 랜덤 회피
            rel  = T_pos - C_pos;
            dist = norm(rel);

            if dist < targetConfig.panic_range
                update_interval = targetConfig.random_update_interval;
                max_turn_rate   = deg2rad(targetConfig.random_turn_rate_max_deg);

                if ~targetState.panic_active
                    targetState.panic_active = true;
                    targetState.just_updated_turn = true; 
                    targetState.random_turn_rate = (2*rand - 1) * max_turn_rate;
                    targetState.last_random_update_time = t_now;
                else
                    if (t_now - targetState.last_random_update_time) >= update_interval
                        targetState.random_turn_rate       = (2*rand - 1) * max_turn_rate;
                        targetState.last_random_update_time = t_now;
                        targetState.just_updated_turn       = true; 
                    else
                        targetState.just_updated_turn       = false;
                    end
                end

                targetState.evasion_offset = targetState.evasion_offset ...
                                             + targetState.random_turn_rate * dt;
                
                max_offset = deg2rad(targetConfig.max_evasion_offset_deg);
                targetState.evasion_offset = max(min(targetState.evasion_offset, ...
                                                      max_offset), -max_offset);
            else
                targetState.panic_active = false;
                targetState.just_updated_turn = false;
                
                max_step = deg2rad(targetConfig.random_turn_rate_max_deg) * dt;
                targetState.evasion_offset = targetState.evasion_offset ...
                    - sign(targetState.evasion_offset) ...
                    * min(abs(targetState.evasion_offset), max_step);
            end

        case 3
            % LOS Perpendicular (실시간 시선각 추적 연속 회피 기동)
            rel  = T_pos - C_pos;
            dist = norm(rel);
 
            if dist < targetConfig.los_perp_range
 
                if ~targetState.panic_active
                    % 1000m 범위 최초 진입 순간
                    targetState.panic_active = true;
                    targetState.just_updated_turn = true; 
                    targetState.last_random_update_time = t_now;
                    
                    if rand > 0.5
                        targetState.evasion_sign = 1;   
                    else
                        targetState.evasion_sign = -1;  
                    end
                else
                    % 1초 간격 시각화용 타이머 업데이트
                    marker_interval = 1.0; 
                    if (t_now - targetState.last_random_update_time) >= marker_interval
                        targetState.just_updated_turn       = true;
                        targetState.last_random_update_time = t_now;
                    else
                        targetState.just_updated_turn       = false;
                    end
                end
 
                % 🔴 고정된 panic_perp_dir 대신, 실시간 시선 수직 벡터 계산
                los_dir  = rel / dist;
                perp_dir = [-los_dir(2); los_dir(1)] * targetState.evasion_sign;
                
                perp_heading = atan2(perp_dir(2), perp_dir(1));
                
                % 1) 목표 오프셋 계산
                desired_offset = wrapAngleLocal(perp_heading - base_heading);

                % 2) 선회율 제한 적용 (180 deg/s)
                max_step = deg2rad(targetConfig.los_perp_turn_rate_deg) * dt;
                offset_error = wrapAngleLocal(desired_offset - targetState.evasion_offset);
                
                targetState.evasion_offset = targetState.evasion_offset ...
                    + max(min(offset_error, max_step), -max_step);
                
                % 3) 🔴 최대 회피 오프셋 한계를 180도로 확장하여 90도 락 해제
                max_offset_dynamic = deg2rad(180); 
                targetState.evasion_offset = max(min(targetState.evasion_offset, ...
                                                      max_offset_dynamic), -max_offset_dynamic);
            else
                % 범위 벗어남: 원복
                targetState.panic_active   = false;
                targetState.just_updated_turn = false;
                
                max_step = deg2rad(targetConfig.los_perp_turn_rate_deg) * dt;
                targetState.evasion_offset = targetState.evasion_offset ...
                    - sign(targetState.evasion_offset) ...
                    * min(abs(targetState.evasion_offset), max_step);
            end

        case 4
            % Optimal Evasion (Collision Triangle)
            rel  = T_pos - C_pos;
            dist = norm(rel);
 
            if dist < targetConfig.collision_range
 
                if ~targetState.panic_active
                    targetState.panic_active = true;
                    targetState.just_updated_turn = true;
                    targetState.last_random_update_time = t_now;
                else
                    % 1초 간격 시각화용 타이머 업데이트
                    marker_interval = 1.0;
                    if (t_now - targetState.last_random_update_time) >= marker_interval
                        targetState.just_updated_turn       = true;
                        targetState.last_random_update_time = t_now;
                    else
                        targetState.just_updated_turn       = false;
                    end
                end
 
                % Chaser 속도 단위벡터 및 실시간 LOS 계산
                C_vel_dir = C_vel / norm(C_vel);
                los_dir = rel / dist;
 
                C_vel_los  = dot(C_vel_dir, los_dir) * los_dir;
                C_vel_perp = C_vel_dir - C_vel_los;
 
                collision_dir = los_dir + C_vel_perp;
                if norm(collision_dir) > 1e-6
                    collision_dir = collision_dir / norm(collision_dir);
                else
                    collision_dir = los_dir;
                end
 
                % Collision point 반대 방향 탈출
                escape_dir      = -collision_dir;
                escape_heading = atan2(escape_dir(2), escape_dir(1));
                desired_offset = wrapAngleLocal(escape_heading - base_heading);
                
                max_step        = deg2rad(targetConfig.collision_turn_rate_deg) * dt;
                offset_error = wrapAngleLocal(desired_offset - targetState.evasion_offset);
                
                targetState.evasion_offset = targetState.evasion_offset ...
                    + max(min(offset_error, max_step), -max_step);
                
                % 🔴 최대 회피 오프셋 한계를 180도로 확장하여 90도 락 해제
                max_offset_dynamic = deg2rad(180); 
                targetState.evasion_offset = max(min(targetState.evasion_offset, ...
                                                      max_offset_dynamic), -max_offset_dynamic);
            else
                % 범위 밖: 원복
                targetState.panic_active = false;
                targetState.just_updated_turn = false;
                
                max_step = deg2rad(targetConfig.collision_turn_rate_deg) * dt;
                targetState.evasion_offset = targetState.evasion_offset ...
                    - sign(targetState.evasion_offset) ...
                    * min(abs(targetState.evasion_offset), max_step);
            end
 
        otherwise
            error("Unknown target mode: %d", target_mode);
    end

    desired_heading = wrapAngleLocal(base_heading + targetState.evasion_offset);
    heading_error   = wrapAngleLocal(desired_heading - T_heading);

    % 회피 선회 제한 적용
    max_heading_rate = deg2rad(max(targetConfig.los_perp_turn_rate_deg, ...
                                   targetConfig.collision_turn_rate_deg));
    heading_step = max(min(heading_error, max_heading_rate * dt), ...
                              -max_heading_rate * dt);
    T_heading = T_heading + heading_step;
    
    T_vel = T_speed * [cos(T_heading); sin(T_heading)];
    T_pos = T_pos + T_vel * dt;
end
 
function ang = wrapAngleLocal(ang)
    ang = atan2(sin(ang), cos(ang));
end