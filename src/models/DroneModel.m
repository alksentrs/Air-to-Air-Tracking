classdef DroneModel < handle
    %DRONEMODEL Constant-velocity 2D drone model.
    %   Implements the CV model:
    %       p_t(k+1) = p_t(k) + v_t dt,  v_t = const.
    
    properties
        position    % drone position p_t in {W}, 2x1
        velocity    % drone velocity v_t in {W}, 2x1
    end
    
    methods
        function obj = DroneModel(p0, v0)
            %DRONEMODEL Construct from initial position and velocity.
            arguments
                p0 (2,1) double
                v0 (2,1) double
            end
            obj.position = p0;
            obj.velocity = v0;
        end
        
        function step(obj, dt)
            %STEP Propagate drone state forward by dt under CV motion.
            obj.position = obj.position + obj.velocity * dt;
        end
        
        function state = getState(obj)
            %GETSTATE Return current state in a struct.
            state.position = obj.position;
            state.velocity = obj.velocity;
        end
    end
end

