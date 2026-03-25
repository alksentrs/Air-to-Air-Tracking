classdef RadarMeasurementModel
    %RADARMEASUREMENTMODEL Nonlinear range/azimuth for side-looking inward radar.
    %   z = [rho; phi] with phi = atan2(y_r, x_r) in radar frame (x = inward normal).
    %   Depends on aircraft position p_a and orbit center (moving sensor).
    
    methods (Static)
        function z = h(x_target, p_aircraft, center)
            %H Measurement function h(x): world target state -> radar [rho; phi].
            arguments
                x_target (4,1) double
                p_aircraft (2,1) double
                center (2,1) double
            end
            p_t = x_target(1:2);
            p_rel = p_t - p_aircraft(:);
            p_radar = CoordinateUtils.worldToRadarFrame(p_rel, p_aircraft, center);
            x_r = p_radar(1);
            y_r = p_radar(2);
            rho = hypot(x_r, y_r);
            phi = atan2(y_r, x_r);
            z = [rho; phi];
        end
        
        function H = jacobianH(x_target, p_aircraft, center)
            %JACOBIANH Analytic H = dh/dx (2 x 4), x = [px; py; vx; vy].
            arguments
                x_target (4,1) double
                p_aircraft (2,1) double
                center (2,1) double
            end
            p_t = x_target(1:2);
            p_rel = p_t - p_aircraft(:);
            p_radar = CoordinateUtils.worldToRadarFrame(p_rel, p_aircraft, center);
            x_r = p_radar(1);
            y_r = p_radar(2);
            rho = hypot(x_r, y_r);
            rho = max(rho, 1e-6);
            
            n_in = CoordinateUtils.inwardNormal(p_aircraft, center);
            t = CoordinateUtils.tangentVector(p_aircraft, center);
            
            drho_dp = (x_r * n_in + y_r * t) / rho;
            dphi_dp = (-y_r / rho^2) * n_in + (x_r / rho^2) * t;
            
            H = [drho_dp(:).', 0, 0;
                 dphi_dp(:).', 0, 0];
        end
        
        function ok = verifyJacobianNumerical(x_target, p_aircraft, center, relTol, eps_x)
            %VERIFYJACOBIANNUMERICAL Compare analytic H to finite-difference (debug).
            if nargin < 4 || isempty(relTol)
                relTol = ProcessingConfig.default().jacobianRelTol;
            end
            if nargin < 5 || isempty(eps_x)
                eps_x = ProcessingConfig.default().numericalJacEpsX;
            end
            H = RadarMeasurementModel.jacobianH(x_target, p_aircraft, center);
            Hn = RadarMeasurementModel.numericalJacobianH(x_target, p_aircraft, center, eps_x);
            err = max(abs(H(:) - Hn(:)) ./ max(1e-9, abs(Hn(:))));
            ok = err < relTol;
        end
        
        function Hn = numericalJacobianH(x_target, p_aircraft, center, eps_x)
            if nargin < 4 || isempty(eps_x)
                eps_x = ProcessingConfig.default().numericalJacEpsX;
            end
            z0 = RadarMeasurementModel.h(x_target, p_aircraft, center);
            Hn = zeros(2, 4);
            for j = 1:4
                dx = zeros(4, 1);
                dx(j) = eps_x;
                z1 = RadarMeasurementModel.h(x_target + dx, p_aircraft, center);
                Hn(:, j) = (z1 - z0) / eps_x;
            end
        end
    end
end
