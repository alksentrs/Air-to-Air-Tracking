function run_stage2_radar_generation()
%RUN_STAGE2_RADAR_GENERATION Stage 2: radar measurements from ground truth.
%   Runs Stage 1 simulation, then generates range/azimuth detections
%   from the side-looking radar (FOV and range gating). Plots and validates.

    % Ensure project src (and subfolders) is on the MATLAB path first.
    thisFile = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));
    srcPath = fullfile(projectRoot, 'src');
    addpath(genpath(srcPath), '-begin');
    rehash path;

    % Stage 1: ground truth
    cfg = SimulationConfig.default();
    scenario = Scenario(cfg);
    simulator = Simulator(cfg, scenario);
    results = simulator.run();

    t = results.t;
    N = numel(t);
    numDrones = cfg.numDrones;
    pA = results.aircraftPos;
    dronePos = results.dronePos;
    center = cfg.aircraftCenter;

    % Radar sensor (side-looking, inward normal)
    if isempty(which('RadarSensor', '-all'))
        error('RadarSensor not found. Ensure %s is on the path.', fullfile(srcPath, 'sensors'));
    end
    radar = RadarSensor.default();
    fprintf('Radar FOV half-angle = %.2f deg (beamwidth = %.2f deg)\n', ...
        rad2deg(radar.fovHalfAngle), 2*rad2deg(radar.fovHalfAngle));

    % Per-target measurement logging
    radarResults.detected = false(numDrones, N);
    radarResults.rho_true = zeros(numDrones, N);
    radarResults.phi_true = zeros(numDrones, N);
    radarResults.rho_meas = NaN(numDrones, N);
    radarResults.phi_meas = NaN(numDrones, N);

    for k = 1:N
        p_a = pA(:, k);
        for i = 1:numDrones
            p_t = dronePos{i}(:, k);
            m = radar.measure(p_a, center, p_t);
            radarResults.detected(i, k) = m.detected;
            radarResults.rho_true(i, k) = m.rho;
            radarResults.phi_true(i, k) = m.phi;
            radarResults.rho_meas(i, k) = m.rho_meas;
            radarResults.phi_meas(i, k) = m.phi_meas;
        end
    end

    % Visualization
    viz = TrajectoryVisualizer();
    viz.plotTrajectories(results);

    plotter = MeasurementPlotter();
    plotter.plot(t, radarResults);

    validate_stage2(t, radarResults, cfg);
end

function validate_stage2(t, radarResults, cfg)
%VALIDATE_STAGE2 Basic Stage 2 validation: geometry, FOV, angle wrap.

    detected = radarResults.detected;
    rho_true = radarResults.rho_true;
    phi_true = radarResults.phi_true;
    rho_meas = radarResults.rho_meas;
    phi_meas = radarResults.phi_meas;
    numDrones = size(detected, 1);

    fprintf('Stage 2 validation:\n');

    % Range positive when computed
    fprintf('  Range: min = %.2f m, max = %.2f m\n', min(rho_true(:)), max(rho_true(:)));

    % Azimuth in [-pi, pi]
    fprintf('  Azimuth: min = %.4f rad, max = %.4f rad\n', min(phi_true(:)), max(phi_true(:)));

    % Detection counts
    for i = 1:numDrones
        n = nnz(detected(i, :));
        fprintf('  Target %d: detected %d / %d time steps\n', i, n, size(detected, 2));
    end

    % When detected, |phi| must be within FOV (sanity check)
    if any(detected(:))
        maxAbsPhiWhenDetected = max(abs(phi_true(detected)));
        fprintf('  Max |azimuth| when detected = %.4f rad (%.2f deg)\n', ...
            maxAbsPhiWhenDetected, rad2deg(maxAbsPhiWhenDetected));
    end

    % When detected, noisy should be finite
    for i = 1:numDrones
        idx = detected(i, :);
        if any(idx)
            assert(all(isfinite(rho_meas(i, idx))), 'rho_meas must be finite when detected');
            assert(all(isfinite(phi_meas(i, idx))), 'phi_meas must be finite when detected');
        end
    end

    % Revisit period hint (orbit T = 2*pi*R/v_a)
    T_orbit = 2 * pi * cfg.aircraftRadius / cfg.aircraftSpeed;
    fprintf('  Orbit period (revisit) ≈ %.2f s\n', T_orbit);
end
