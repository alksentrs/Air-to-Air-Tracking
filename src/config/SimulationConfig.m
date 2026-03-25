classdef SimulationConfig
    %SIMULATIONCONFIG Configuration for Stage 1 dynamic simulation.
    %   Centralizes all tunable parameters for the dynamic simulation.
    %   No numerical values should be hardcoded in the simulation logic.
    
    properties (SetAccess = immutable)
        T_end          % total simulation time [s]
        dt             % time step [s]
        numDrones      % number of drones
    end
    
    properties (SetAccess = immutable)
        aircraftSpeed   % aircraft speed v_a [m/s]
        aircraftRadius  % circular path radius R [m]
        theta0          % initial aircraft heading θ_a(0) [rad]
        aircraftCenter  % center of circular path in {W}, 2x1
    end
    
    properties (SetAccess = immutable)
        droneInitPos    % initial drone positions in {W}, 2 x numDrones
        droneInitVel    % initial drone velocities in {W}, 2 x numDrones
    end
    
    methods
        function obj = SimulationConfig(T_end, dt, numDrones, ...
                aircraftSpeed, aircraftRadius, theta0, aircraftCenter, ...
                droneInitPos, droneInitVel)
            % Constructor with explicit parameters.
            
            arguments
                T_end (1,1) double {mustBePositive}
                dt (1,1) double {mustBePositive}
                numDrones (1,1) double {mustBeInteger, mustBeNonnegative}
                aircraftSpeed (1,1) double {mustBePositive}
                aircraftRadius (1,1) double {mustBePositive}
                theta0 (1,1) double
                aircraftCenter (2,1) double = [0; 0]
                droneInitPos double = []
                droneInitVel double = []
            end
            
            if numDrones == 0
                droneInitPos = zeros(2, 0);
                droneInitVel = zeros(2, 0);
            else
                if isempty(droneInitPos)
                    droneInitPos = zeros(2, numDrones);
                end
                if isempty(droneInitVel)
                    droneInitVel = zeros(2, numDrones);
                end
                if ~isequal(size(droneInitPos), [2, numDrones])
                    error('SimulationConfig:InvalidDroneInitPos', ...
                        'droneInitPos must be 2 x numDrones.');
                end
                if ~isequal(size(droneInitVel), [2, numDrones])
                    error('SimulationConfig:InvalidDroneInitVel', ...
                        'droneInitVel must be 2 x numDrones.');
                end
            end
            
            obj.T_end = T_end;
            obj.dt = dt;
            obj.numDrones = numDrones;
            
            obj.aircraftSpeed = aircraftSpeed;
            obj.aircraftRadius = aircraftRadius;
            obj.theta0 = theta0;
            obj.aircraftCenter = aircraftCenter;
            
            obj.droneInitPos = droneInitPos;
            obj.droneInitVel = droneInitVel;
        end
    end
    
    methods (Static)
        function cfg = default()
            %DEFAULT Create a default Stage 1 simulation configuration.
            %   The values follow the SDD typical parameters and provide
            %   a simple multi-drone scenario for testing.
            
            T_end = 2000;
            dt = 0.1;
            numDrones = 3;
            
            aircraftSpeed = 300.0;
            aircraftRadius = 1000.0;
            theta0 = pi;
            aircraftCenter = [0.0; 0.0];
            
            % Typical drone speed magnitude ≈ 30 m/s (SDD).
            vMag = -30.0;
            
            % Arrange drones with different headings; none at orbit center so each has one detection passage.
            droneInitPos = [56510.34482758621   23793.10344827586   01379.31034482759;
   31034.48275862069  -20800.689655172414   59310.34482758621];
            
            droneHeadings = [pi/6, -pi/3, pi/2];
            droneInitVel = zeros(2, numDrones);
            for i = 1:numDrones
                droneInitVel(:, i) = vMag * [cos(droneHeadings(i)); sin(droneHeadings(i))];
            end
            
            cfg = SimulationConfig( ...
                T_end, dt, numDrones, ...
                aircraftSpeed, aircraftRadius, theta0, aircraftCenter, ...
                droneInitPos, droneInitVel);
        end
    end
end

