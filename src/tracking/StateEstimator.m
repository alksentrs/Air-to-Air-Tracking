classdef (Abstract) StateEstimator < handle
    %STATEESTIMATOR Abstract interface for recursive state estimators (Stage 3+).
    %   Implementations must support prediction and measurement updates.
    
    methods (Abstract)
        predict(obj, dt)
        %PREDICT Propagate state estimate and covariance by dt [s].
        
        update(obj, z_meas)
        %UPDATE Incorporate measurement vector z_meas (skipped if empty/NaN per convention).
    end
end
