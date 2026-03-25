classdef TrackingPlotter
    %TRACKINGPLOTTER Stage 3 figures: true vs estimated, errors, innovations.
    
    methods (Static)
        function plotAll(t, dronePos, loggers, trackIds, measXY)
            %PLOTALL Multi-target overview: all truth drones + variable track estimates.
            arguments
                t (1,:) double
                dronePos cell
                loggers cell
                trackIds double = []
                measXY (2,:) double = nan(2, 0)
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

            if ~isempty(measXY) && size(measXY, 1) == 2 && any(isfinite(measXY(:)))
                plot(measXY(1, :), measXY(2, :), 'k.', 'MarkerSize', 6, 'DisplayName', 'Measurements');
            end

            legD = arrayfun(@(d) sprintf('Truth drone %d', d), 1:nd, 'UniformOutput', false);
            legT = arrayfun(@(i) sprintf('Est track %d', trackIds(i)), 1:nTr, 'UniformOutput', false);
            if ~isempty(measXY) && size(measXY, 1) == 2 && any(isfinite(measXY(:)))
                legend([legD, legT, {'Measurements'}], 'Location', 'best');
            else
                legend([legD, legT], 'Location', 'best');
            end
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

            hasTruth = all(isfinite(p_true(:)));
            if hasTruth
                subplot(3, 1, 1);
                plot(t, logger.pos_err, 'b-', 'LineWidth', 1.2);
                ylabel('Position error [m]');
                title(sprintf('Track %d: position error vs truth', targetId));
                grid on;

                spRho = 2;
                spPhi = 3;
            else
                sgtitle(sprintf('Track %d: innovations (no truth matched)', targetId));
                spRho = 1;
                spPhi = 2;
            end

            subplot(3 - ~hasTruth, 1, spRho);
            idx = logger.had_update;
            plot(t(idx), logger.nu_rho(idx), '.-', 'MarkerSize', 8);
            grid on;
            ylabel('Innovation \rho [m]');

            subplot(3 - ~hasTruth, 1, spPhi);
            plot(t(idx), logger.nu_phi(idx), '.-', 'MarkerSize', 8);
            grid on;
            xlabel('Time [s]');
            ylabel('Innovation \phi [rad]');
        end
    end
end
