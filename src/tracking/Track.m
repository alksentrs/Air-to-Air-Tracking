classdef Track < handle
    %TRACK Holder for one target's EKF and log (Stage 3). Extensible for multi-target logic later.
    
    properties
        targetId
        ekf      % EKFTracker
        logger   % TrackingLogger
    end
    
    methods
        function obj = Track(targetId, ekf, logger)
            arguments
                targetId (1,1) double {mustBeInteger, mustBePositive}
                ekf
                logger
            end
            obj.targetId = targetId;
            obj.ekf = ekf;
            obj.logger = logger;
        end
    end
end
