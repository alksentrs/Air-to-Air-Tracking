classdef ProcessingConfig
    %PROCESSINGCONFIG Configuration for radar + tracking processing (Stage 2/3).
    %   Centralizes tunable parameters for measurement generation, gating,
    %   and EKF tracker tuning.
    
    properties (SetAccess = immutable)
        % Radar measurement / detection model
        radarSigmaRange      % [m]
        radarSigmaAzimuth    % [rad]
        radarFovHalfAngle    % [rad]
        radarRhoMin          % [m]
        radarRhoMax          % [m]
        
        % Tracking / association
        gnnGateChi2Thresh    % chi-square threshold for NIS gate (2 DOF)
        sigmaAccel           % [m/s^2] process noise intensity for CV model
        
        % EKF initialization
        initPosVarScaleFromRangeVar  % multiplier on range variance for px/py init
        initVelStd                  % [m/s] 1-sigma initial velocity uncertainty
        
        % Numerical verification (debug)
        jacobianRelTol      % relative tolerance for H check
        numericalJacEpsX    % finite difference step for numerical Jacobian
    end
    
    methods
        function obj = ProcessingConfig( ...
                radarSigmaRange, radarSigmaAzimuth, radarFovHalfAngle, radarRhoMin, radarRhoMax, ...
                gnnGateChi2Thresh, sigmaAccel, ...
                initPosVarScaleFromRangeVar, initVelStd, ...
                jacobianRelTol, numericalJacEpsX)
            arguments
                radarSigmaRange (1,1) double {mustBeNonnegative}
                radarSigmaAzimuth (1,1) double {mustBeNonnegative}
                radarFovHalfAngle (1,1) double {mustBePositive}
                radarRhoMin (1,1) double {mustBeNonnegative}
                radarRhoMax (1,1) double {mustBePositive}
                gnnGateChi2Thresh (1,1) double {mustBePositive}
                sigmaAccel (1,1) double {mustBeNonnegative}
                initPosVarScaleFromRangeVar (1,1) double {mustBePositive}
                initVelStd (1,1) double {mustBeNonnegative}
                jacobianRelTol (1,1) double {mustBePositive}
                numericalJacEpsX (1,1) double {mustBePositive}
            end
            if radarRhoMax <= radarRhoMin
                error('ProcessingConfig:InvalidRadarRange', 'radarRhoMax must be > radarRhoMin.');
            end
            
            obj.radarSigmaRange = radarSigmaRange;
            obj.radarSigmaAzimuth = radarSigmaAzimuth;
            obj.radarFovHalfAngle = radarFovHalfAngle;
            obj.radarRhoMin = radarRhoMin;
            obj.radarRhoMax = radarRhoMax;
            
            obj.gnnGateChi2Thresh = gnnGateChi2Thresh;
            obj.sigmaAccel = sigmaAccel;
            
            obj.initPosVarScaleFromRangeVar = initPosVarScaleFromRangeVar;
            obj.initVelStd = initVelStd;
            
            obj.jacobianRelTol = jacobianRelTol;
            obj.numericalJacEpsX = numericalJacEpsX;
        end
    end
    
    methods (Static)
        function cfg = default()
            %DEFAULT Defaults matching current Stage 2/3 behavior.
            radarSigmaRange = 5.0;             % m
            radarSigmaAzimuth = deg2rad(1.0);  % rad
            radarFovHalfAngle = deg2rad(1);    % +/- 1 deg
            radarRhoMin = 50.0;                % m
            radarRhoMax = 60000.0;             % m
            
            % Association gate on d^2 = nu' * inv(S) * nu, with 2 measurement dims
            gnnGateChi2Thresh = 20.0;
            
            % CV model process noise (tunable)
            sigmaAccel = 2.0;                  % m/s^2
            
            % EKF init covariance tuning
            initPosVarScaleFromRangeVar = 4.0;
            initVelStd = 30.0;                 % m/s
            
            % Debug-only numerical jacobian checking
            jacobianRelTol = 1e-5;
            numericalJacEpsX = 1e-4;
            
            cfg = ProcessingConfig( ...
                radarSigmaRange, radarSigmaAzimuth, radarFovHalfAngle, radarRhoMin, radarRhoMax, ...
                gnnGateChi2Thresh, sigmaAccel, ...
                initPosVarScaleFromRangeVar, initVelStd, ...
                jacobianRelTol, numericalJacEpsX);
        end
        
        function radar = buildRadarSensor(cfg)
            arguments
                cfg (1,1) ProcessingConfig
            end
            radar = RadarSensor( ...
                cfg.radarSigmaRange, cfg.radarSigmaAzimuth, cfg.radarFovHalfAngle, ...
                cfg.radarRhoMin, cfg.radarRhoMax);
        end
        
        function R = measurementNoiseCov(cfg)
            arguments
                cfg (1,1) ProcessingConfig
            end
            R = diag([cfg.radarSigmaRange^2, cfg.radarSigmaAzimuth^2]);
        end
    end
end

