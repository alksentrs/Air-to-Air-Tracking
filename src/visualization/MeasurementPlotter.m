classdef MeasurementPlotter
    %MEASUREMENTPLOTTER Stage 2 radar measurement visualization.
    %   Range vs time, azimuth vs time, detection timeline per target.
    
    methods
        function plot(obj, t, radarResults)
            %PLOT Range, azimuth, and detection visibility vs time.
            %   radarResults.detected (numDrones x N), rho_true, phi_true, rho_meas, phi_meas.
            numDrones = size(radarResults.detected, 1);
            colors = lines(max(numDrones, 1));
            
            % Range vs time
            figure('Name', 'Stage 2: Range vs time');
            hold on; grid on;
            xlabel('Time [s]');
            ylabel('Range [m]');
            title('Drone_{radar} range: true vs noisy');
            for i = 1:numDrones
                plot(t, radarResults.rho_true(i, :), '-', 'Color', colors(i, :), 'DisplayName', sprintf('Target %d true', i));
                idx = radarResults.detected(i, :);
                if any(idx)
                    plot(t(idx), radarResults.rho_meas(i, idx), '.', 'Color', colors(i, :), 'MarkerSize', 8, 'DisplayName', sprintf('Target %d meas', i));
                end
            end
            legend('Location', 'best');
            
            % Azimuth vs time
            figure('Name', 'Stage 2: Azimuth vs time');
            hold on; grid on;
            xlabel('Time [s]');
            ylabel('Azimuth [rad]');
            title('Drone_{radar} azimuth: true vs noisy');
            for i = 1:numDrones
                plot(t, radarResults.phi_true(i, :), '-', 'Color', colors(i, :), 'DisplayName', sprintf('Target %d true', i));
                idx = radarResults.detected(i, :);
                if any(idx)
                    plot(t(idx), radarResults.phi_meas(i, idx), '.', 'Color', colors(i, :), 'MarkerSize', 8, 'DisplayName', sprintf('Target %d meas', i));
                end
            end
            legend('Location', 'best');
            
            % Detection timeline (visible / not visible)
            figure('Name', 'Stage 2: Detection timeline');
            for i = 1:numDrones
                subplot(numDrones, 1, i);
                stairs(t, double(radarResults.detected(i, :)), 'Color', colors(i, :), 'LineWidth', 1);
                grid on;
                ylabel(sprintf('Target %d', i));
                ylim([-0.1, 1.1]);
                yticks([0, 1]);
                yticklabels({'no', 'yes'});
            end
            xlabel('Time [s]');
            sgtitle('Detection timeline (in FOV = yes)');
        end
    end
end
