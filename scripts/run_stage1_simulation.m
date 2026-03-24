function run_stage1_simulation()
%RUN_STAGE1_SIMULATION Entry point for Stage 1 dynamic simulation.
%   Sets up configuration, runs the simulator, visualizes trajectories,
%   and performs basic validation checks against the Stage 1 checklist.

    % Ensure src is on the MATLAB path.
    thisFile = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));
    srcPath = fullfile(projectRoot, 'src');
    if all(arrayfun(@(p) ~strcmpi(p, srcPath), strsplit(path, pathsep)))
        addpath(genpath(srcPath));
    end

    cfg = SimulationConfig.default();
    scenario = Scenario(cfg);
    simulator = Simulator(cfg, scenario);

    results = simulator.run();

    viz = TrajectoryVisualizer();
    viz.plotTrajectories(results);

    validate_stage1(results);
end

function validate_stage1(results)
%VALIDATE_STAGE1 Perform basic Stage 1 validation checks.

    t = results.t;
    pA = results.aircraftPos;
    thetaA = results.aircraftHeading;
    dronePos = results.dronePos;
    cfg = results.config;

    dt = cfg.dt;

    % Aircraft speed and circular trajectory.
    dpA = diff(pA, 1, 2);
    speedSamples = vecnorm(dpA, 2, 1) / dt;
    meanSpeed = mean(speedSamples);
    stdSpeed = std(speedSamples);

    radii = vecnorm(pA - cfg.aircraftCenter, 2, 1);
    meanRadius = mean(radii);
    stdRadius = std(radii);

    fprintf('Aircraft speed: mean = %.3f m/s, std = %.3f m/s (target %.3f m/s)\n', ...
        meanSpeed, stdSpeed, cfg.aircraftSpeed);
    fprintf('Aircraft radius: mean = %.3f m, std = %.3f m (target %.3f m)\n', ...
        meanRadius, stdRadius, cfg.aircraftRadius);

    % Drone constant-velocity straight-line trajectories.
    numDrones = numel(dronePos);
    for i = 1:numDrones
        pD = dronePos{i};
        dpD = diff(pD, 1, 2);
        vSamples = dpD / dt;
        speedD = vecnorm(vSamples, 2, 1);
        meanSpeedD = mean(speedD);
        stdSpeedD = std(speedD);

        fprintf('Drone %d speed: mean = %.3f m/s, std = %.3f m/s\n', ...
            i, meanSpeedD, stdSpeedD);
    end

    % Basic time vector consistency.
    expectedN = floor(cfg.T_end / cfg.dt) + 1;
    if numel(t) ~= expectedN
        warning('Time vector length (%d) does not match expected (%d).', ...
            numel(t), expectedN);
    end

    % Simple heading evolution sanity check.
    dTheta = diff(thetaA);
    meanOmega = mean(dTheta) / dt;
    fprintf('Aircraft mean angular rate ≈ %.5f rad/s\n', meanOmega);
end

