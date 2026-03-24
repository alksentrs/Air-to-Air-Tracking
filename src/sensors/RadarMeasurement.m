classdef RadarMeasurement
    %RADARMEASUREMENT Single time-step radar observation for one target.
    %   Holds detection flag, true (rho, phi), and noisy (rho_meas, phi_meas).
    %   For missed detections, rho_meas and phi_meas are NaN.
    
    properties
        detected   % true if target is in FOV and within range
        rho        % true range [m]
        phi        % true azimuth [rad], in [-pi, pi]
        rho_meas   % noisy range (NaN if not detected)
        phi_meas   % noisy azimuth (NaN if not detected)
    end
    
    methods
        function obj = RadarMeasurement(detected, rho, phi, rho_meas, phi_meas)
            arguments
                detected (1,1) logical
                rho (1,1) double
                phi (1,1) double
                rho_meas (1,1) double
                phi_meas (1,1) double
            end
            obj.detected = detected;
            obj.rho = rho;
            obj.phi = phi;
            obj.rho_meas = rho_meas;
            obj.phi_meas = phi_meas;
        end
        
        function s = toStruct(obj)
            s = struct('detected', obj.detected, 'rho', obj.rho, 'phi', obj.phi, ...
                'rho_meas', obj.rho_meas, 'phi_meas', obj.phi_meas);
        end
    end
    
    methods (Static)
        function m = createMissed(rho, phi)
            %CREATEMISSED Measurement for a missed detection (true rho/phi, meas NaN).
            m = RadarMeasurement(false, rho, phi, NaN, NaN);
        end
        
        function m = createDetected(rho, phi, rho_meas, phi_meas)
            %CREATEDETECTED Measurement for a detection with noise.
            m = RadarMeasurement(true, rho, phi, rho_meas, phi_meas);
        end
    end
end
