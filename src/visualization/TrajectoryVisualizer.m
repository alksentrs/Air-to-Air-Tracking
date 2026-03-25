classdef TrajectoryVisualizer
    %TRAJECTORYVISUALIZER Plots Stage 1 trajectories and heading.
    
    properties
        headingStride = 175   % plot heading arrow every N samples
        headingScale = 500   % arrow length scaling factor
        antennaScale = 250  % arrow length scaling factor
    end
    
    methods
        function obj = TrajectoryVisualizer(varargin)
            %TRAJECTORYVISUALIZER Optional name-value configuration.
            if mod(nargin, 2) ~= 0
                error('TrajectoryVisualizer:InvalidArgs', ...
                    'Use name-value pairs for configuration.');
            end
            for i = 1:2:nargin
                name = varargin{i};
                value = varargin{i+1};
                if isprop(obj, name)
                    obj.(name) = value;
                else
                    error('TrajectoryVisualizer:UnknownProperty', ...
                        'Unknown property "%s".', name);
                end
            end
        end
        
        function plotTrajectories(obj, results)
            %PLOTTRAJECTORIES Plot aircraft and drone trajectories and heading.
            %
            % results.t               : 1 x N
            % results.aircraftPos     : 2 x N
            % results.aircraftHeading : 1 x N
            % results.dronePos        : 1 x numDrones cell, each 2 x N
            
            t = results.t;
            pA = results.aircraftPos;
            thetaA = results.aircraftHeading;
            dronePos = results.dronePos;
            numDrones = numel(dronePos);
            
            figure('Name', 'Stage 2: Trajectories');
            hold on;
            grid on;
            axis equal;
            xlabel('x [m]');
            ylabel('y [m]');
            title('Stage 1 Dynamic Simulation: Aircraft and Drones');
            
            plot(pA(1, :), pA(2, :), 'b-', 'LineWidth', 1.5);
            % Mark the start of the aircraft trajectory.
            plot(pA(1, 1), pA(2, 1), 'o', 'Color', 'r', 'MarkerFaceColor', 'r', ...
                'MarkerSize', 8, 'LineWidth', 2);
            
            colors = lines(max(numDrones, 1));
            for i = 1:numDrones
                pD = dronePos{i};
                plot(pD(1, :), pD(2, :), '-', 'Color', colors(i, :), 'LineWidth', 1.2);
                % Mark the start of each drone trajectory.
                plot(pD(1, 1), pD(2, 1), 'o', 'Color', 'r', 'MarkerFaceColor', 'r', ...
                    'MarkerSize', 7, 'LineWidth', 2);
            end
            
            % Heading visualization (only first sample for a clean plot).
            idx = 1;
            dirs = [cos(thetaA(idx)); sin(thetaA(idx))];
            u = obj.headingScale * dirs(1);
            v = obj.headingScale * dirs(2);
            quiver(pA(1, idx), pA(2, idx), u, v, 2, 'k');

            % Antenna / radar boresight visualization at first sample.
            if isfield(results, 'aircraftInward')
                n_in = results.aircraftInward(:, idx);
                ux_rad = obj.antennaScale * n_in(1);
                vy_rad = obj.antennaScale * n_in(2);
                quiver(pA(1, idx), pA(2, idx), ux_rad, vy_rad, 2, 'r', 'LineWidth', 1.5);
            end
            
            legendEntries = [{'Aircraft'}, arrayfun(@(i) sprintf('Drone %d', i), 1:numDrones, 'UniformOutput', false)];
            legend(legendEntries{:}, 'Location', 'best');
        end
    end
end

