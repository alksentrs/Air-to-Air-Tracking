classdef Simulator
    %SIMULATOR Stage 1 dynamic simulation engine.
    %   Advances the aircraft and drone models over time and logs history.
    
    properties (SetAccess = immutable)
        config      % SimulationConfig
        scenario    % Scenario
    end
    
    properties (SetAccess = private)
        t                      % time vector 1 x N
        aircraftPosHistory     % 2 x N
        aircraftHeadingHistory % 1 x N
        aircraftInwardHistory  % 2 x N (radar boresight / inward normal)
        dronePosHistory        % 1 x numDrones cell, each 2 x N
    end
    
    methods
        function obj = Simulator(cfg, scenario)
            %SIMULATOR Construct from SimulationConfig and Scenario.
            arguments
                cfg (1,1) SimulationConfig
                scenario (1,1) Scenario
            end
            
            obj.config = cfg;
            obj.scenario = scenario;
        end
        
        function results = run(obj)
            %RUN Execute the dynamic simulation loop.
            
            cfg = obj.config;
            sc = obj.scenario;
            dt = cfg.dt;
            
            obj.t = 0:dt:cfg.T_end;
            N = numel(obj.t);
            numDrones = cfg.numDrones;
            
            obj.aircraftPosHistory = zeros(2, N);
            obj.aircraftHeadingHistory = zeros(1, N);
            obj.aircraftInwardHistory = zeros(2, N);
            obj.dronePosHistory = cell(1, numDrones);
            for i = 1:numDrones
                obj.dronePosHistory{i} = zeros(2, N);
            end
            
            for k = 1:N
                % Log current states.
                obj.aircraftPosHistory(:, k) = sc.aircraft.position;
                obj.aircraftHeadingHistory(:, k) = sc.aircraft.theta;
                obj.aircraftInwardHistory(:, k) = sc.aircraft.inwardNormal;
                
                for i = 1:numDrones
                    obj.dronePosHistory{i}(:, k) = sc.drones(i).position;
                end
                
                % Advance states, except at final time step.
                if k < N
                    sc.aircraft.step(dt);
                    for i = 1:numDrones
                        sc.drones(i).step(dt);
                    end
                end
            end
            
            results = struct();
            results.t = obj.t;
            results.aircraftPos = obj.aircraftPosHistory;
            results.aircraftHeading = obj.aircraftHeadingHistory;
            results.aircraftInward = obj.aircraftInwardHistory;
            results.dronePos = obj.dronePosHistory;
            results.config = cfg;
        end
    end
end

