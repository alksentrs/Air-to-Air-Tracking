classdef AircraftModel < handle
    %AIRCRAFTMODEL Circular motion aircraft model in 2D.
    %   Implements the aircraft dynamics specified in the SDD:
    %       θ_a(k+1) = θ_a(k) + ω_a dt
    %       p_a(k+1) = p_a(k) + v_a [cos(θ_a); sin(θ_a)] dt
    
    properties
        position    % aircraft position p_a in {W}, 2x1
        theta       % heading angle θ_a [rad]
    end
    
    properties (SetAccess = private)
        speed       % v_a [m/s]
        radius      % R [m]
        omega       % ω_a = v_a / R [rad/s]
        center      % center of circular path in {W}, 2x1
        inwardNormal % unit inward normal in {W}, 2x1 (toward center)
        tangent      % unit tangent in {W}, 2x1 (velocity direction)
    end
    
    methods
        function obj = AircraftModel(cfg)
            %AIRCRAFTMODEL Construct from SimulationConfig.
            arguments
                cfg (1,1) SimulationConfig
            end
            
            obj.speed = cfg.aircraftSpeed;
            obj.radius = cfg.aircraftRadius;
            obj.omega = obj.speed / obj.radius;
            obj.center = cfg.aircraftCenter;
            
            obj.theta = cfg.theta0;
            obj.position = obj.center + obj.radius * [sin(obj.theta); -cos(obj.theta)];
            obj.updateGeometryVectors();
        end
        
        function step(obj, dt)
            %STEP Propagate aircraft state forward by dt.
            %   Uses the discrete-time model from the SDD.
            
            obj.theta = obj.theta + obj.omega * dt;
            direction = [cos(obj.theta); sin(obj.theta)];
            obj.position = obj.position + obj.speed * direction * dt;
            obj.updateGeometryVectors();
        end
        
        function state = getState(obj)
            %GETSTATE Return current state in a struct.
            state.position = obj.position;
            state.theta = obj.theta;
            state.speed = obj.speed;
            state.inwardNormal = obj.inwardNormal;
            state.tangent = obj.tangent;
        end
    end

    methods (Access = private)
        function updateGeometryVectors(obj)
            % Keep inward normal and tangent consistent with orbit geometry.
            obj.inwardNormal = CoordinateUtils.inwardNormal(obj.position, obj.center);
            obj.tangent = CoordinateUtils.tangentVector(obj.position, obj.center);
        end
    end
end

