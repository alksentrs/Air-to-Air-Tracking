classdef TrackingPlotter
    %TRACKINGPLOTTER Stage 3 figures: true vs estimated, errors, innovations.
    
    methods (Static)
        function plotAll(t, dronePos, loggers, trackIds)
            %PLOTALL Multi-target overview: all truth drones + variable track estimates.
            arguments
                t (1,:) double
                dronePos cell
                loggers cell
                trackIds double = []
            end
            nTr = numel(loggers);
            nd = numel(dronePos);
            if isempty(trackIds)
                trackIds = 1:nTr;
            end
            figure('Name', 'Stage 3: Tracking');
            hold on; grid on; axis equal;
            xlabel('x [m]'); ylabel('y [m]');
            title('Truth drones vs estimated tracks');
            cols = lines(max([nTr, nd, 1]));
            for d = 1:nd
                pT = dronePos{d};
                plot(pT(1, :), pT(2, :), '--', 'Color', cols(mod(d-1, size(cols,1))+1, :), 'LineWidth', 1.0);
            end
            for i = 1:nTr
                xh = loggers{i}.x_hat;
                plot(xh(1, :), xh(2, :), '-', 'Color', cols(mod(i-1, size(cols,1))+1, :), 'LineWidth', 1.5);
            end
            legD = arrayfun(@(d) sprintf('Truth drone %d', d), 1:nd, 'UniformOutput', false);
            legT = arrayfun(@(i) sprintf('Est track %d', trackIds(i)), 1:nTr, 'UniformOutput', false);
            legend([legD{:}, legT{:}], 'Location', 'best');
        end
        
        function plotTargetDetail(t, p_true, logger, targetId)
            %PLOTTARGETDETAIL Position error (if truth finite) and innovations for one track.
            arguments
                t (1,:) double
                p_true (2,:) double
                logger (1,1) TrackingLogger
                targetId (1,1) double
            end
            figure('Name', sprintf('Stage 3: Track %d detail', targetId));
            
            subplot(3, 1, 1);
            if all(isfinite(p_true(:)))
                plot(t, logger.pos_err, 'b-', 'LineWidth', 1.2);
                ylabel('Position error [m]');
                title(sprintf('Track %d: position error vs truth', targetId));
            else
                text(0.5, 0.5, 'No per-drone truth bound (use optimal matching for RMSE)', ...
                    'HorizontalAlignment', 'center', 'Units', 'normalized');
                title(sprintf('Track %d: position error N/A', targetId));
            end
            grid on;
            
            subplot(3, 1, 2);
            idx = logger.had_update;
            plot(t(idx), logger.nu_rho(idx), '.-', 'MarkerSize', 8);
            grid on;
            ylabel('Innovation \rho [m]');
            
            subplot(3, 1, 3);
            plot(t(idx), logger.nu_phi(idx), '.-', 'MarkerSize', 8);
            grid on;
            xlabel('Time [s]');
            ylabel('Innovation \phi [rad]');
        end
    end
end
