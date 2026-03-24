classdef Scenario
    %SCENARIO Container for Stage 1 dynamic simulation entities.
    %   Builds the aircraft and drone models from a SimulationConfig.
    
    properties (SetAccess = immutable)
        config      % SimulationConfig
        aircraft    % AircraftModel
        drones      % array of DroneModel, 1 x numDrones
    end
    
    methods
        function obj = Scenario(cfg)
            %SCENARIO Construct from SimulationConfig.
            arguments
                cfg (1,1) SimulationConfig
            end
            
            obj.config = cfg;
            
            obj.aircraft = AircraftModel(cfg);
            
            num = cfg.numDrones;
            if num > 0
                % Preallocate handle array using repmat from a single instance.
                dm = DroneModel([0; 0], [0; 0]);
                obj.drones = repmat(dm, 1, num);
                for i = 1:num
                    p0 = cfg.droneInitPos(:, i);
                    v0 = cfg.droneInitVel(:, i);
                    obj.drones(i) = DroneModel(p0, v0);
                end
            end
        end
    end
end

