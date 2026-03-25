classdef GlobalNearestNeighborAssociator
    %GLOBALNEARESTNEIGHBORASSOCIATOR GNN with Mahalanobis gating (no drone labels).
    %   Phase A only: assign measurements to existing initialized tracks.
    %   Unassociated measurements are returned as newMeasIdx for track birth (caller).
    
    methods (Static)
        function [pairs, newMeasIdx] = assign(measList, tracks, p_aircraft, center, R, gateThresh)
            %ASSIGN pairs: struct array measIdx, trackIdx for updates to existing tracks.
            %   newMeasIdx: row vector of meas indices needing new Track creation + init.
            arguments
                measList cell
                tracks cell
                p_aircraft (2,1) double
                center (2,1) double
                R (2,2) double
                gateThresh (1,1) double {mustBePositive} = ProcessingConfig.default().gnnGateChi2Thresh
            end
            M = numel(measList);
            N = numel(tracks);
            pairs = struct('measIdx', {}, 'trackIdx', {});
            
            if M == 0
                newMeasIdx = [];
                return;
            end
            if N == 0
                newMeasIdx = 1:M;
                return;
            end
            % gateThresh is a chi-square threshold for NIS (2 DOF).
            
            initIdx = GlobalNearestNeighborAssociator.collectInitTrackIndices(tracks);
            nInit = numel(initIdx);
            
            assignedMeas = false(M, 1);
            
            if nInit > 0
                C = inf(M, nInit);
                for i = 1:M
                    z = measList{i}.z(:);
                    for col = 1:nInit
                        j = initIdx(col);
                        ekf = tracks{j}.ekf;
                        z_pred = RadarMeasurementModel.h(ekf.getState(), p_aircraft, center);
                        nu = z - z_pred;
                        nu(2) = atan2(sin(nu(2)), cos(nu(2)));
                        H = RadarMeasurementModel.jacobianH(ekf.getState(), p_aircraft, center);
                        Pk = ekf.getCovariance();
                        S = H * Pk * H' + R;
                        d2 = nu' * (S \ nu);
                        if d2 <= gateThresh
                            C(i, col) = d2;
                        end
                    end
                end
                pairsA = GlobalNearestNeighborAssociator.solveAssignment(C);
                for p = 1:numel(pairsA)
                    mi = pairsA(p).measIdx;
                    col = pairsA(p).trackIdx;
                    tj = initIdx(col);
                    pairs(end+1).measIdx = mi; %#ok<AGROW>
                    pairs(end).trackIdx = tj;
                    assignedMeas(mi) = true;
                end
            end
            
            newMeasIdx = find(~assignedMeas)';
        end
        
        function idx = collectInitTrackIndices(tracks)
            idx = [];
            for j = 1:numel(tracks)
                if tracks{j}.ekf.isInitialized()
                    idx(end+1) = j; %#ok<AGROW>
                end
            end
        end
        
        function pairs = solveAssignment(C)
            %SOLVEASSIGNMENT Min-cost one-to-one assignment; matchpairs if available else greedy.
            arguments
                C double
            end
            M = size(C, 1);
            N = size(C, 2);
            if M == 0 || N == 0
                pairs = struct('measIdx', {}, 'trackIdx', {});
                return;
            end
            if exist('matchpairs', 'file') == 2
                maxFin = max(C(isfinite(C)));
                if isempty(maxFin)
                    pairs = struct('measIdx', {}, 'trackIdx', {});
                    return;
                end
                uc = maxFin + 1e6;
                C2 = C;
                C2(~isfinite(C2)) = maxFin + 1e12;
                try
                    Massign = matchpairs(C2, uc);
                catch
                    Massign = GlobalNearestNeighborAssociator.greedyAssignment(C);
                end
            else
                Massign = GlobalNearestNeighborAssociator.greedyAssignment(C);
            end
            nPairs = size(Massign, 1);
            pairs = struct('measIdx', {}, 'trackIdx', {});
            for p = 1:nPairs
                i = Massign(p, 1);
                j = Massign(p, 2);
                if isfinite(C(i, j))
                    pairs(end+1).measIdx = i; %#ok<AGROW>
                    pairs(end).trackIdx = j;
                end
            end
        end
        
        function Massign = greedyAssignment(C)
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
        
        function p = measurementToWorld(z, p_aircraft, center)
            rho = z(1);
            phi = z(2);
            n_in = CoordinateUtils.inwardNormal(p_aircraft, center);
            t = CoordinateUtils.tangentVector(p_aircraft, center);
            x_r = rho * cos(phi);
            y_r = rho * sin(phi);
            p_rel = n_in * x_r + t * y_r;
            p = p_aircraft(:) + p_rel;
        end
    end
end
