
% plotFunction
% -------------------------------------------------------------------------
% Plot 관련 기능을 한 파일에 모은 함수.
%
% 사용법
%   plotData = plotFunction("initRealtime", C_pos, T_pos, dt, plot_update_interval)
%   plotData = plotFunction("updateRealtime", plotData, k, C_hist, T_hist, target_mode, T_heading, C_heading, panic2_log, panic3_log, panic4_log))
%   plotFunction("summary", time, C_hist, T_hist, R_hist, a_hist,
%   mode_hist, T_goal, guide_hist, panic2_log, panic3_log, panic4_log)
% -------------------------------------------------------------------------
function varargout = plotFunction(action, varargin)
    action = char(action);

    switch lower(action)

        case 'initrealtime'
            C_pos                = varargin{1};
            T_pos                = varargin{2};
            T_goal               = varargin{3};
            dt                   = varargin{4};
            plot_update_interval = varargin{5};

            plotData.update_steps = max(1, round(plot_update_interval / dt));

            plotData.fig_anim = figure;
            hold on; grid on; axis equal;
            xlabel('X Position [m]');
            ylabel('Y Position [m]');
            title('2D Chaser-Target Realtime Demo');

            % 목적지 별
            plotData.hT_goal = plot(T_goal(1), T_goal(2), 'p', ...
                'MarkerSize', 15, 'MarkerFaceColor', [1 0.6 0], ...
                'MarkerEdgeColor', [0.8 0.4 0], 'LineWidth', 1.5);

            % start markers
            plotData.hC_start = plot(C_pos(1), C_pos(2), 'bo', ...
                'MarkerSize', 8, 'LineWidth', 2);
            plotData.hT_start = plot(T_pos(1), T_pos(2), 'ro', ...
                'MarkerSize', 8, 'LineWidth', 2);

            % trajectory lines
            plotData.hC_traj = plot(C_pos(1), C_pos(2), 'b', 'LineWidth', 2);
            plotData.hT_traj = plot(T_pos(1), T_pos(2), 'r', 'LineWidth', 2);

            % current position markers
            plotData.hC_now = plot(C_pos(1), C_pos(2), 'bx', ...
                'MarkerSize', 10, 'LineWidth', 2);
            plotData.hT_now = plot(T_pos(1), T_pos(2), 'rx', ...
                'MarkerSize', 10, 'LineWidth', 2);

            % 회피 범위 원
            theta_circle = linspace(0, 2*pi, 360);
            r_alert = 1000;
            cx = T_pos(1) + r_alert * cos(theta_circle);
            cy = T_pos(2) + r_alert * sin(theta_circle);
            plotData.hAlert = plot(cx, cy, '--', ...
                'Color', [1 0.6 0.6], 'LineWidth', 1.2);
            plotData.r_alert   = r_alert;
            plotData.theta_circle = theta_circle;

            % 회피 발동시 표시용 핸들 (Mode 2, 3, 4)
            plotData.hPanic2 = plot(nan, nan, 'm^', ...
                'MarkerSize', 7, 'MarkerFaceColor', 'm', 'LineStyle', 'none');
            plotData.hPanic3 = plot(nan, nan, 'gd', ...
                'MarkerSize', 7, 'MarkerFaceColor', 'g', 'LineStyle', 'none');
            plotData.hPanic4 = plot(nan, nan, 'co', ...
                'MarkerSize', 7, 'MarkerFaceColor', 'c', 'LineStyle', 'none');

            % 회피 방향 화살표 및 LOS
            plotData.hArrowT = quiver(T_pos(1), T_pos(2), 0, 0, ...
                'Color', [0.9 0.1 0.1], 'LineWidth', 2, ...
                'MaxHeadSize', 2, 'AutoScale', 'off');
            plotData.hArrowC = quiver(C_pos(1), C_pos(2), 0, 0, ...
                'Color', [0.1 0.1 0.9], 'LineWidth', 2, ...
                'MaxHeadSize', 2, 'AutoScale', 'off');
            plotData.hLOS = plot([C_pos(1) T_pos(1)], [C_pos(2) T_pos(2)], ...
                'k--', 'LineWidth', 1.0);

            %  Legend 설정
            legend([plotData.hC_traj,  plotData.hT_traj, ...
                    plotData.hC_start, plotData.hT_start, ...
                    plotData.hC_now,   plotData.hT_now, ...
                    plotData.hT_goal,  plotData.hAlert, ...
                    plotData.hPanic2,  plotData.hPanic3, plotData.hPanic4, ...
                    plotData.hArrowT, plotData.hArrowC], ...
                   {'Chaser traj',    'Target traj', ...
                    'Chaser start',   'Target start', ...
                    'Chaser current', 'Target current', ...
                    'Target goal',    'Alert 1000m', ...
                    'Mode2 evasion',  'Mode3 evasion', 'Mode4 evasion', ...
                    'Target heading', 'Chaser heading'}, ...
                   'Location', 'northwest');   

            drawnow;
            varargout = {plotData};

        case 'updaterealtime'
            plotData   = varargin{1};
            k          = varargin{2};
            C_hist     = varargin{3};
            T_hist     = varargin{4};
            target_mode = varargin{5};
            T_heading  = varargin{6};
            C_heading  = varargin{7};
            panic2_log = varargin{8};   % Mode2 기록
            panic3_log = varargin{9};   % Mode3 기록 
            panic4_log = varargin{10};  % Mode4 기록 

            if mod(k-1, plotData.update_steps) == 0 || k == size(C_hist, 2)

                set(plotData.hC_traj, 'XData', C_hist(1,1:k), 'YData', C_hist(2,1:k));
                set(plotData.hT_traj, 'XData', T_hist(1,1:k), 'YData', T_hist(2,1:k));
                set(plotData.hC_now,  'XData', C_hist(1,k),   'YData', C_hist(2,k));
                set(plotData.hT_now,  'XData', T_hist(1,k),   'YData', T_hist(2,k));

                % 회피 원 이동
                cx = T_hist(1,k) + plotData.r_alert * cos(plotData.theta_circle);
                cy = T_hist(2,k) + plotData.r_alert * sin(plotData.theta_circle);
                set(plotData.hAlert, 'XData', cx, 'YData', cy);

                dist = norm(C_hist(:,k) - T_hist(:,k));
                if dist < plotData.r_alert
                    set(plotData.hAlert, 'Color', [1 0.0 0.0], 'LineWidth', 2.0);
                else
                    set(plotData.hAlert, 'Color', [1 0.6 0.6], 'LineWidth', 1.2);
                end

                %  화살표 길이 (거리 비례, 최대 500m)
                arrow_len = min(dist * 0.15, 500);

                %  Target heading 화살표
                set(plotData.hArrowT, ...
                    'XData', T_hist(1,k), 'YData', T_hist(2,k), ...
                    'UData', arrow_len * cos(T_heading), ...
                    'VData', arrow_len * sin(T_heading));

                %  Chaser heading 화살표
                set(plotData.hArrowC, ...
                    'XData', C_hist(1,k), 'YData', C_hist(2,k), ...
                    'UData', arrow_len * cos(C_heading), ...
                    'VData', arrow_len * sin(C_heading));

                %  LOS 선 (chaser → target)
                set(plotData.hLOS, ...
                    'XData', [C_hist(1,k), T_hist(1,k)], ...
                    'YData', [C_hist(2,k), T_hist(2,k)]);

                %  모드별 회피 시각화 업데이트
                if ~isempty(panic2_log)
                    set(plotData.hPanic2, 'XData', panic2_log(:,1), 'YData', panic2_log(:,2));
                end
                if ~isempty(panic3_log)
                    set(plotData.hPanic3, 'XData', panic3_log(:,1), 'YData', panic3_log(:,2));
                end
                if ~isempty(panic4_log)
                    set(plotData.hPanic4, 'XData', panic4_log(:,1), 'YData', panic4_log(:,2));
                end

                % 축 자동 확장
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

            varargout = {plotData};

        case 'summary'
            time      = varargin{1};
            C_hist    = varargin{2};
            T_hist    = varargin{3};
            R_hist    = varargin{4};
            a_hist    = varargin{5};
            mode_hist = varargin{6};
            T_goal    = varargin{7};
            guide_hist = varargin{8};   
            panic2_log = varargin{9}; 
            panic3_log = varargin{10};
            panic4_log = varargin{11};

            % ── 그래프 1: Chaser - PP & PNG Step Trajectory ──────────────────────────────
            figure;
            hold on; grid on; axis equal;

            %  PurePursuit 구간
            pp_idx = find(guide_hist == 1);
            pp_steps = length(pp_idx);
            h_pp = [];
            if ~isempty(pp_idx)
                h_pp= plot(C_hist(1, pp_idx), C_hist(2, pp_idx), '.', ...
                    'Color', [0.2 0.8 0.2], 'MarkerSize', 5);
            end

            %  PNG 구간 
            png_idx = find(guide_hist == 2);
            png_steps = length(png_idx);
            h_png = [];
            if ~isempty(png_idx)
                h_png= plot(C_hist(1, png_idx), C_hist(2, png_idx), '.', ...
                    'Color', [0.2 0.2 1.0], 'MarkerSize', 5);
            end

            %  Target 궤적 
            h_T_traj = plot(T_hist(1,:), T_hist(2,:), 'r', 'LineWidth', 1.5);
            h_C_start = plot(C_hist(1,1),   C_hist(2,1),   'bo', 'MarkerSize', 8,  'LineWidth', 2);
            h_T_start = plot(T_hist(1,1),   T_hist(2,1),   'ro', 'MarkerSize', 8,  'LineWidth', 2);
            h_C_end   = plot(C_hist(1,end), C_hist(2,end), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
            h_T_end   = plot(T_hist(1,end), T_hist(2,end), 'rx', 'MarkerSize', 10, 'LineWidth', 2);

            % 목적지 별
            h_goal = plot(T_goal(1), T_goal(2), 'p', 'MarkerSize', 15, 'MarkerFaceColor', [1 0.6 0], ...
                'MarkerEdgeColor', [0.8 0.4 0], 'LineWidth', 1.5);

            %  Mode 2,3,4 랜덤회피 발동 점
            h_panic2 = [];
            if ~isempty(panic2_log)
                h_panic2 = plot(panic2_log(:,1), panic2_log(:,2), 'm^', 'MarkerSize', 7, 'MarkerFaceColor', 'm', 'LineStyle', 'none');
            end

            h_panic3 = [];
            if ~isempty(panic3_log)
                h_panic3 = plot(panic3_log(:,1), panic3_log(:,2), 'gd', 'MarkerSize', 7, 'MarkerFaceColor', 'g', 'LineStyle', 'none');
            end

            h_panic4 = [];
            if ~isempty(panic4_log)
                h_panic4 = plot(panic4_log(:,1), panic4_log(:,2), 'co', 'MarkerSize', 7, 'MarkerFaceColor', 'c', 'LineStyle', 'none');
            end

            % 범례 구성 (PP 및 PNG 스텝 수 포함)
            h_list     = [h_T_traj, h_C_start, h_T_start, h_C_end, h_T_end, h_goal];
            label_list = {'Target traj', 'Chaser start', 'Target start', ...
                          'Chaser end',  'Target end',   'Target goal'};

            if ~isempty(h_pp)
                h_list     = [h_pp, h_list];
                label_list = [{sprintf('Chaser (PurePursuit): %d steps', pp_steps)}, label_list];
            end
            if ~isempty(h_png)
                % PP 라벨 다음 위치에 PNG 라벨 결합
                h_list     = [h_png, h_list];
                label_list = [{sprintf('Chaser (PNG): %d steps', png_steps)}, label_list];
            end
            if ~isempty(h_panic2)
                h_list     = [h_list, h_panic2];
                label_list = [label_list, {'Mode2 evasion trigger'}];
            end
            if ~isempty(h_panic3)
                h_list     = [h_list, h_panic3];
                label_list = [label_list, {'Mode3 evasion trigger'}];
            end
            if ~isempty(h_panic4)
                h_list     = [h_list, h_panic4];
                label_list = [label_list, {'Mode4 evasion trigger'}];
            end

            legend(h_list, label_list, 'Location', 'northwest');
            xlabel('X [m]'); ylabel('Y [m]');
            title('Chaser - PP & PNG Step Trajectory');


            % ── 그래프 2: Range ──────────────────────────────────────────
            figure;
            plot(time, R_hist, 'LineWidth', 2);
            grid on;
            xlabel('Time [s]'); ylabel('Range [m]');
            title('Distance between Chaser and Target');

            varargout = {};

        otherwise
            error("Unknown plotFunction action: %s", action);
    end
end