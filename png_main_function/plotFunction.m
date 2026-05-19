function varargout = plotFunction(action, varargin)
% plotFunction
% -------------------------------------------------------------------------
% Plot 관련 기능을 한 파일에 모은 함수.
%
% 사용법
%   plotData = plotFunction("initRealtime", C_pos, T_pos, dt, plot_update_interval)
%   plotData = plotFunction("updateRealtime", plotData, k, C_hist, T_hist)
%   plotFunction("summary", time, C_hist, T_hist, R_hist, a_hist, mode_hist, panic_hist)
% -------------------------------------------------------------------------

    action = char(action);

    switch lower(action)

        case 'initrealtime'
            C_pos = varargin{1};
            T_pos = varargin{2};
            dt = varargin{3};
            plot_update_interval = varargin{4};

            plotData.update_steps = max(1, round(plot_update_interval / dt));

            plotData.fig_anim = figure;
            hold on; grid on; axis equal;
            xlabel('X Position [m]');
            ylabel('Y Position [m]');
            title('2D Chaser-Target Realtime Demo');

            % start markers
            plotData.hC_start = plot(C_pos(1), C_pos(2), 'bo', ...
                'MarkerSize', 8, 'LineWidth', 2);
            plotData.hT_start = plot(T_pos(1), T_pos(2), 'ro', ...
                'MarkerSize', 8, 'LineWidth', 2);

            % trajectory line handles
            plotData.hC_traj = plot(C_pos(1), C_pos(2), 'b', 'LineWidth', 2);
            plotData.hT_traj = plot(T_pos(1), T_pos(2), 'r', 'LineWidth', 2);

            % current position markers
            plotData.hC_now = plot(C_pos(1), C_pos(2), 'bx', ...
                'MarkerSize', 10, 'LineWidth', 2);
            plotData.hT_now = plot(T_pos(1), T_pos(2), 'rx', ...
                'MarkerSize', 10, 'LineWidth', 2);

            legend([plotData.hC_traj, plotData.hT_traj, ...
                    plotData.hC_start, plotData.hT_start, ...
                    plotData.hC_now, plotData.hT_now], ...
                   {'Chaser trajectory', 'Target trajectory', ...
                    'Chaser start', 'Target start', ...
                    'Chaser current', 'Target current'}, ...
                   'Location', 'best');

            drawnow;
            varargout = {plotData};

        case 'updaterealtime'
            plotData = varargin{1};
            k = varargin{2};
            C_hist = varargin{3};
            T_hist = varargin{4};

            if mod(k-1, plotData.update_steps) == 0 || k == size(C_hist, 2)
                set(plotData.hC_traj, 'XData', C_hist(1,1:k), 'YData', C_hist(2,1:k));
                set(plotData.hT_traj, 'XData', T_hist(1,1:k), 'YData', T_hist(2,1:k));

                set(plotData.hC_now, 'XData', C_hist(1,k), 'YData', C_hist(2,k));
                set(plotData.hT_now, 'XData', T_hist(1,k), 'YData', T_hist(2,k));

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

            varargout = {plotData};

        case 'summary'
            time = varargin{1};
            C_hist = varargin{2};
            T_hist = varargin{3};
            R_hist = varargin{4};
            a_hist = varargin{5};
            mode_hist = varargin{6};
            panic_hist = varargin{7};

            % 1) Final trajectory plot
            figure;
            plot(C_hist(1,:), C_hist(2,:), 'b', 'LineWidth', 2); hold on;
            plot(T_hist(1,:), T_hist(2,:), 'r', 'LineWidth', 2);

            plot(C_hist(1,1), C_hist(2,1), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
            plot(T_hist(1,1), T_hist(2,1), 'ro', 'MarkerSize', 8, 'LineWidth', 2);

            plot(C_hist(1,end), C_hist(2,end), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
            plot(T_hist(1,end), T_hist(2,end), 'rx', 'MarkerSize', 10, 'LineWidth', 2);

            grid on; axis equal;
            xlabel('X Position [m]');
            ylabel('Y Position [m]');
            legend('Chaser trajectory', 'Target trajectory', ...
                   'Chaser start', 'Target start', ...
                   'Chaser end', 'Target end', ...
                   'Location', 'best');
            title('2D Chaser-Target Trajectory');

            % 2) Range plot
            figure;
            plot(time, R_hist, 'LineWidth', 2);
            grid on;
            xlabel('Time [s]');
            ylabel('Range [m]');
            title('Distance between Chaser and Target');

            % 3) Acceleration command plot
            figure;
            plot(time, a_hist, 'LineWidth', 2);
            grid on;
            xlabel('Time [s]');
            ylabel('Acceleration Command [m/s^2]');
            title('PN-like Acceleration Command');

            % 4) Target mode history plot
            figure;
            stairs(time, mode_hist, 'LineWidth', 2);
            grid on;
            xlabel('Time [s]');
            ylabel('Target Mode');
            yticks([1 2 3 4 5]);
            ylim([0.5 5.5]);
            title('Target Behavior Mode History');

            % 5) Panic state history plot
            figure;
            stairs(time, panic_hist, 'LineWidth', 2);
            grid on;
            xlabel('Time [s]');
            ylabel('Panic Active');
            yticks([0 1]);
            ylim([-0.2 1.2]);
            title('Panic Escape State');

            varargout = {};

        otherwise
            error("Unknown plotFunction action: %s", action);
    end
end
