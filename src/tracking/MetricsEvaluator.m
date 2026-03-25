classdef MetricsEvaluator
    %METRICSEVALUATOR Position RMSE and optional NEES for tracking validation.
    
    methods (Static)
        function rmse = positionRmse(p_true_hist, p_hat_hist)
            %POSITIONRMSE Scalar RMSE over time where both positions finite.
            arguments
                p_true_hist (2,:) double
                p_hat_hist (2,:) double
            end
            d = p_true_hist - p_hat_hist;
            err2 = sum(d.^2, 1);
            m = isfinite(err2);
            if ~any(m)
                rmse = NaN;
                return;
            end
            rmse = sqrt(mean(err2(m)));
        end
        
        function [rmseVals, trackIdx, droneIdx] = optimalTrackDroneRmse(trackLoggers, dronePos)
            %OPTIMALTRACKDRONERMSE Match tracks to truth drones by min mean squared error; report RMSE per pair.
            arguments
                trackLoggers cell
                dronePos cell
            end
            Nt = numel(trackLoggers);
            Nd = numel(dronePos);
            if Nt == 0 || Nd == 0
                rmseVals = [];
                trackIdx = [];
                droneIdx = [];
                return;
            end
            N = size(trackLoggers{1}.x_hat, 2);
            C = inf(Nt, Nd);
            for t = 1:Nt
                xh = trackLoggers{t}.x_hat(1:2, :);
                for d = 1:Nd
                    pt = dronePos{d};
                    sse = 0;
                    cnt = 0;
                    for k = 1:N
                        if all(isfinite(xh(:, k)))
                            sse = sse + sum((xh(:, k) - pt(:, k)).^2);
                            cnt = cnt + 1;
                        end
                    end
                    if cnt > 0
                        C(t, d) = sse / cnt;
                    end
                end
            end
            maxFin = max(C(isfinite(C)));
            if isempty(maxFin)
                rmseVals = [];
                trackIdx = [];
                droneIdx = [];
                return;
            end
            C2 = C;
            C2(~isfinite(C2)) = maxFin + 1e12;
            uc = maxFin + 1e6;
            try
                Massign = matchpairs(C2, uc);
            catch
                Massign = MetricsEvaluator.greedyRowsCols(C);
            end
            nP = size(Massign, 1);
            rmseVals = [];
            trackIdx = [];
            droneIdx = [];
            for p = 1:nP
                ti = Massign(p, 1);
                di = Massign(p, 2);
                if ~isfinite(C(ti, di))
                    continue;
                end
                trackIdx(end+1, 1) = ti; %#ok<AGROW>
                droneIdx(end+1, 1) = di; %#ok<AGROW>
                rmseVals(end+1, 1) = MetricsEvaluator.positionRmse(dronePos{di}, trackLoggers{ti}.x_hat(1:2, :)); %#ok<AGROW>
            end
        end
        
        function Massign = greedyRowsCols(C)
            M = size(C, 1);
            N = size(C, 2);
            usedR = false(M, 1);
            usedC = false(N, 1);
            rows = [];
            cols = [];
            for iter = 1:min(M, N)
                best = inf;
                bi = 0;
                bj = 0;
                for i = 1:M
                    if usedR(i), continue; end
                    for j = 1:N
                        if usedC(j), continue; end
                        c = C(i, j);
                        if isfinite(c) && c < best
                            best = c;
                            bi = i;
                            bj = j;
                        end
                    end
                end
                if bi == 0
                    break;
                end
                usedR(bi) = true;
                usedC(bj) = true;
                rows(end+1, 1) = bi; %#ok<AGROW>
                cols(end+1, 1) = bj; %#ok<AGROW>
            end
            Massign = [rows, cols];
        end
        
        function nees = neesPosition(x_err, P_pos)
            %NEESPOSITION NEES using position substate only (2x2 covariance blocks).
            arguments
                x_err (4,:) double
                P_pos (2,2,:) double
            end
            N = size(x_err, 2);
            nees = nan(1, N);
            for k = 1:N
                e = x_err(1:2, k);
                Pk = P_pos(:, :, k);
                if all(isfinite(e)) && all(isfinite(Pk(:)))
                    nees(k) = e' * (Pk \ e);
                end
            end
        end
    end
end
