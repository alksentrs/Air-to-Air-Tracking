classdef TrackingLogger < handle
    %TRACKINGLOGGER Per-target EKF run history for Stage 3 analysis and plots.
    
    properties
        t            % 1 x N time [s]
        x_hat        % 4 x N [px; py; vx; vy] (NaN before initialized)
        P_trace      % 1 x N trace(P)
        P_hist       % 4 x 4 x N covariance (NaN before initialized)
        pos_err      % 1 x N ||p_hat - p_true|| [m]
        nu_rho       % 1 x N range innovation (NaN if no update)
        nu_phi       % 1 x N azimuth innovation (NaN if no update)
        had_update   % 1 x N logical (measurement used after predict)
    end
    
    methods
        function obj = TrackingLogger(N)
            arguments
                N (1,1) double {mustBeInteger, mustBePositive}
            end
            obj.t = nan(1, N);
            obj.x_hat = nan(4, N);
            obj.P_trace = nan(1, N);
            obj.P_hist = nan(4, 4, N);
            obj.pos_err = nan(1, N);
            obj.nu_rho = nan(1, N);
            obj.nu_phi = nan(1, N);
            obj.had_update = false(1, N);
        end
        
        function record(obj, k, t_k, xk, Pk, nu, hadUpd, p_true)
            arguments
                obj (1,1) TrackingLogger
                k (1,1) double {mustBeInteger, mustBePositive}
                t_k (1,1) double
                xk (4,1) double
                Pk (4,4) double
                nu (2,1) double
                hadUpd (1,1) logical
                p_true (2,1) double = [NaN; NaN]
            end
            obj.t(k) = t_k;
            obj.x_hat(:, k) = xk;
            obj.P_trace(k) = trace(Pk);
            obj.P_hist(:, :, k) = Pk;
            if all(isfinite(xk(1:2))) && all(isfinite(p_true))
                obj.pos_err(k) = norm(xk(1:2) - p_true(:));
            end
            if hadUpd
                obj.nu_rho(k) = nu(1);
                obj.nu_phi(k) = nu(2);
                obj.had_update(k) = true;
            end
        end
    end
end
