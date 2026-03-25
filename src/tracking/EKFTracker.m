classdef EKFTracker < StateEstimator
    %EKFTRACKER Extended Kalman filter for CV target with radar (rho, phi) measurements.
    %   State x = [px; py; vx; vy]. Sensor is moving; h(x) uses aircraft position and orbit center.
    
    properties (SetAccess = private)
        x            % 4 x 1 state estimate
        P            % 4 x 4 covariance
        center       % orbit center in {W} (2x1), for inward radar frame
        dt           % nominal time step [s] (F matrix)
        sigmaAccel   % process noise intensity [m/s^2] for CV model
        initPosVarScaleFromRangeVar  % multiplier on range variance for px/py init
        initVelStd                  % [m/s] 1-sigma initial velocity uncertainty
        F            % 4 x 4 state transition (CV)
        Q            % 4 x 4 process noise covariance
        R            % 2 x 2 measurement noise (range, azimuth)
        initialized  % logical
    end
    
    methods
        function obj = EKFTracker(orbitCenter, dt, sigmaAccel, R_meas, initPosVarScaleFromRangeVar, initVelStd)
            %EKFTRACKER Construct EKF for one target.
            arguments
                orbitCenter (2,1) double
                dt (1,1) double {mustBePositive}
                sigmaAccel (1,1) double {mustBeNonnegative}
                R_meas (2,2) double
                initPosVarScaleFromRangeVar (1,1) double {mustBePositive} = 4.0
                initVelStd (1,1) double {mustBeNonnegative} = 30.0
            end
            obj.center = orbitCenter(:);
            obj.dt = dt;
            obj.sigmaAccel = sigmaAccel;
            obj.initPosVarScaleFromRangeVar = initPosVarScaleFromRangeVar;
            obj.initVelStd = initVelStd;
            obj.F = [1, 0, dt, 0; ...
                     0, 1, 0, dt; ...
                     0, 0, 1, 0; ...
                     0, 0, 0, 1];
            obj.Q = EKFTracker.cvProcessNoise(dt, sigmaAccel);
            obj.R = R_meas;
            obj.x = zeros(4, 1);
            obj.P = eye(4);
            obj.initialized = false;
        end
        
        function tf = isInitialized(obj)
            tf = obj.initialized;
        end
        
        function xhat = getState(obj)
            xhat = obj.x;
        end
        
        function Pk = getCovariance(obj)
            Pk = obj.P;
        end
        
        function initializeFromMeasurement(obj, z_meas, p_aircraft)
            %INITIALIZEFROMMEASUREMENT First update from [rho; phi] and aircraft pose.
            arguments
                obj (1,1) EKFTracker
                z_meas (2,1) double
                p_aircraft (2,1) double
            end
            rho = z_meas(1);
            phi = z_meas(2);
            n_in = CoordinateUtils.inwardNormal(p_aircraft, obj.center);
            t = CoordinateUtils.tangentVector(p_aircraft, obj.center);
            x_r = rho * cos(phi);
            y_r = rho * sin(phi);
            p_rel = n_in * x_r + t * y_r;
            p_t = p_aircraft(:) + p_rel;
            obj.x = [p_t; 0; 0];
            % Large initial uncertainty (position from noisy meas, velocity unknown)
            obj.P = diag([ ...
                obj.R(1,1) * obj.initPosVarScaleFromRangeVar, ...
                obj.R(1,1) * obj.initPosVarScaleFromRangeVar, ...
                obj.initVelStd^2, ...
                obj.initVelStd^2]);
            obj.initialized = true;
        end
        
        function predict(obj, dt)
            %PREDICT Time update for step dt [s].
            if ~obj.initialized
                return;
            end
            if abs(dt - obj.dt) < 1e-12
                obj.x = obj.F * obj.x;
                obj.P = obj.F * obj.P * obj.F' + obj.Q;
            else
                Fk = [1, 0, dt, 0; 0, 1, 0, dt; 0, 0, 1, 0; 0, 0, 0, 1];
                Qk = EKFTracker.cvProcessNoise(dt, obj.sigmaAccel);
                obj.x = Fk * obj.x;
                obj.P = Fk * obj.P * Fk' + Qk;
            end
        end
        
        function update(obj, meas)
            %UPDATE Measurement update. meas struct: .z (2x1 rho,phi), .p_aircraft (2x1), .t (time, optional).
            %   Skip if meas empty or z non-finite (missed detection).
            if nargin < 2 || isempty(meas)
                return;
            end
            z = meas.z(:);
            p_aircraft = meas.p_aircraft(:);
            if any(~isfinite(z))
                return;
            end
            if ~obj.initialized
                obj.initializeFromMeasurement(z, p_aircraft);
                return;
            end
            
            z_pred = RadarMeasurementModel.h(obj.x, p_aircraft, obj.center);
            nu = z - z_pred;
            nu(2) = atan2(sin(nu(2)), cos(nu(2)));
            
            H = RadarMeasurementModel.jacobianH(obj.x, p_aircraft, obj.center);
            S = H * obj.P * H' + obj.R;
            K = obj.P * H' / S;
            obj.x = obj.x + K * nu;
            Ikh = eye(4) - K * H;
            obj.P = Ikh * obj.P * Ikh' + K * obj.R * K';
        end
        
        function [nu, S] = computeInnovation(obj, z, p_aircraft)
            %COMPUTEINNOVATION For logging: innovation and covariance (after predict).
            z_pred = RadarMeasurementModel.h(obj.x, p_aircraft, obj.center);
            nu = z - z_pred;
            nu(2) = atan2(sin(nu(2)), cos(nu(2)));
            H = RadarMeasurementModel.jacobianH(obj.x, p_aircraft, obj.center);
            S = H * obj.P * H' + obj.R;
        end
    end
    
    methods (Static)
        function Q = cvProcessNoise(dt, sigmaAccel)
            q = sigmaAccel^2;
            dt2 = dt * dt;
            dt3 = dt2 * dt;
            dt4 = dt3 * dt;
            Q = q * [dt4/4, 0,     dt3/2, 0; ...
                     0,     dt4/4, 0,     dt3/2; ...
                     dt3/2, 0,     dt2,   0; ...
                     0,     dt3/2, 0,     dt2];
        end
        
        function ekf = fromRadarSensor(orbitCenter, dt, radar, procCfg)
            %FROMRADARSENSOR Build EKF with R matching RadarSensor noise sigmas.
            arguments
                orbitCenter (2,1) double
                dt (1,1) double
                radar (1,1) RadarSensor
                procCfg (1,1) ProcessingConfig = ProcessingConfig.default()
            end
            R = diag([radar.sigmaRange^2, radar.sigmaAzimuth^2]);
            ekf = EKFTracker(orbitCenter, dt, procCfg.sigmaAccel, R, ...
                procCfg.initPosVarScaleFromRangeVar, procCfg.initVelStd);
        end
    end
end
