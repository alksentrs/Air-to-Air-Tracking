classdef RadarSensor
    %RADARSENSOR Side-looking radar: range and azimuth with sector-limited detection.
    %   Boresight = inward normal. Detection only if target is in the illuminated sector:
    %   in front (x_r > 0), |phi| <= alpha, and rho in [rhoMin, rhoMax].
    %   Measurements: rho = ||p_rel_radar||, phi = atan2(y_r, x_r). Additive Gaussian noise.
    
    properties (SetAccess = immutable)
        sigmaRange    % sigma_rho [m]
        sigmaAzimuth % sigma_phi [rad]
        fovHalfAngle % alpha = Theta_FOV/2 [rad]
        rhoMin       % minimum range [m]
        rhoMax       % maximum range [m]
    end
    
    methods
        function obj = RadarSensor(sigmaRange, sigmaAzimuth, fovHalfAngle, rhoMin, rhoMax)
            %RADARSENSOR Construct with noise and FOV parameters.
            arguments
                sigmaRange (1,1) double {mustBeNonnegative}
                sigmaAzimuth (1,1) double {mustBeNonnegative}
                fovHalfAngle (1,1) double {mustBePositive}
                rhoMin (1,1) double {mustBeNonnegative}
                rhoMax (1,1) double {mustBePositive}
            end
            if rhoMax <= rhoMin
                error('RadarSensor:InvalidRange', 'rhoMax must be > rhoMin.');
            end
            obj.sigmaRange = sigmaRange;
            obj.sigmaAzimuth = sigmaAzimuth;
            obj.fovHalfAngle = fovHalfAngle;
            obj.rhoMin = rhoMin;
            obj.rhoMax = rhoMax;
        end
        
        function m = measure(obj, p_aircraft, center, p_target)
            %MEASURE Compute detection and (true + noisy) range/azimuth for one target.
            %   p_aircraft, center, p_target are 2x1 in inertial frame.
            %   Returns RadarMeasurement (detected or missed).
            %   Detection only when target lies inside the illuminated sector:
            %   in front of radar (x_r > 0), within angular cone |phi| <= alpha, and in range.
            p_rel_w = p_target(:) - p_aircraft(:);
            p_radar = CoordinateUtils.worldToRadarFrame(p_rel_w, p_aircraft, center);
            x_r = p_radar(1);
            y_r = p_radar(2);
            rho = norm(p_radar);
            phi = atan2(y_r, x_r);
            
            % Sector constraint (phi is defined about inward-normal boresight because
            % the radar-frame x-axis is the inward normal).
            inFront = x_r > 0;
            inAngle = abs(phi) <= obj.fovHalfAngle;
            inRange = rho >= obj.rhoMin & rho <= obj.rhoMax;
            detected = inFront && inAngle && inRange;
            
            if ~detected
                m = RadarMeasurement.createMissed(rho, phi);
                return;
            end
            
            rho_meas = rho + obj.sigmaRange * randn();
            phi_meas = phi + obj.sigmaAzimuth * randn();
            phi_meas = atan2(sin(phi_meas), cos(phi_meas)); % wrap to [-pi, pi]
            m = RadarMeasurement.createDetected(rho, phi, rho_meas, phi_meas);
        end
    end
    
    methods (Static)
        function sensor = default(procCfg)
            %DEFAULT Radar with narrow illuminated sector and noise for Stage 2/3.
            %   If procCfg omitted, use ProcessingConfig.default().
            arguments
                procCfg (1,1) ProcessingConfig = ProcessingConfig.default()
            end
            sensor = ProcessingConfig.buildRadarSensor(procCfg);
        end
    end
end
