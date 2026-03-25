function run_stage3_tracking()
%RUN_STAGE3_TRACKING Stage 3: EKF with data association and dynamic track birth.
%   Tracks are created when a detection cannot be associated with any existing
%   (initialized) track. Predict/update iterate over numel(tracks).
%   RMSE vs truth uses evaluation-only optimal track-to-drone matching.
    close all;
    thisFile = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));
    srcPath = fullfile(projectRoot, 'src');
    addpath(genpath(srcPath), '-begin');
    rehash path;

    if ~usejava('desktop')
        set(0, 'DefaultFigureVisible', 'off');
    end
    rng(42);

    cfg = SimulationConfig.default();
    procCfg = ProcessingConfig.default();
    scenario = Scenario(cfg);
    simulator = Simulator(cfg, scenario);
    results = simulator.run();

    t = results.t;
    N = numel(t);
    dt = cfg.dt;
    numDrones = cfg.numDrones;
    pA = results.aircraftPos;
    dronePos = results.dronePos;
    center = cfg.aircraftCenter;

    radar = RadarSensor.default(procCfg);
    R = ProcessingConfig.measurementNoiseCov(procCfg);

    radarResults.detected = false(numDrones, N);
    radarResults.rho_meas = NaN(numDrones, N);
    radarResults.phi_meas = NaN(numDrones, N);

    for k = 1:N
        p_a = pA(:, k);
        for i = 1:numDrones
            p_t = dronePos{i}(:, k);
            m = radar.measure(p_a, center, p_t);
            radarResults.detected(i, k) = m.detected;
            if m.detected
                radarResults.rho_meas(i, k) = m.rho_meas;
                radarResults.phi_meas(i, k) = m.phi_meas;
            end
        end
    end

    xtest = [1000; 500; -10; 5];
    pat = [200; 300];
    okJac = RadarMeasurementModel.verifyJacobianNumerical(xtest, pat, center, procCfg.jacobianRelTol, procCfg.numericalJacEpsX);
    fprintf('Analytic vs numerical H check: %s\n', char(string(okJac)));

    tracks = {};
    nextTrackId = 1;

    for k = 1:N
        p_a = pA(:, k);

        for j = 1:numel(tracks)
            if tracks{j}.ekf.isInitialized()
                tracks{j}.ekf.predict(dt);
            end
        end

        measList = {};
        trueSrc = [];
        for i = 1:numDrones
            if radarResults.detected(i, k)
                s.z = [radarResults.rho_meas(i, k); radarResults.phi_meas(i, k)];
                s.p_aircraft = p_a;
                measList{end+1} = s; %#ok<AGROW>
                trueSrc(end+1) = i;
            end
        end

        if ~isempty(measList)
            perm = randperm(numel(measList));
            measList = measList(perm);
            trueSrc = trueSrc(perm);
        end

        [pairs, newMeasIdx] = GlobalNearestNeighborAssociator.assign(measList, tracks, p_a, center, R, procCfg.gnnGateChi2Thresh);

        nt0 = numel(tracks);
        nuByTrack = cell(nt0, 1);
        hadKalmanByTrack = false(nt0, 1);

        for p = 1:numel(pairs)
            mi = pairs(p).measIdx;
            tj = pairs(p).trackIdx;
            meas = measList{mi};
            ekf = tracks{tj}.ekf;
            if ekf.isInitialized()
                nuByTrack{tj} = ekf.computeInnovation(meas.z, p_a);
                hadKalmanByTrack(tj) = true;
            end
            ekf.update(meas);
        end

        newMeasIdx = sort(newMeasIdx);
        for ii = 1:numel(newMeasIdx)
            mi = newMeasIdx(ii);
            meas = measList{mi};
            logg = TrackingLogger(N);
            ekf = EKFTracker.fromRadarSensor(center, dt, radar, procCfg);
            tracks{end+1} = Track(nextTrackId, ekf, logg); %#ok<AGROW>
            nextTrackId = nextTrackId + 1;
            tracks{end}.ekf.update(meas);
            nuByTrack{end+1} = []; %#ok<AGROW>
            hadKalmanByTrack(end+1) = false;
        end

        nt = numel(tracks);
        for j = 1:nt
            ekf = tracks{j}.ekf;
            logg = tracks{j}.logger;
            if ekf.isInitialized()
                nu = [NaN; NaN];
                hk = false;
                if j <= numel(hadKalmanByTrack) && hadKalmanByTrack(j) && ~isempty(nuByTrack{j})
                    nu = nuByTrack{j}(:);
                    hk = true;
                end
                logg.record(k, t(k), ekf.getState(), ekf.getCovariance(), nu, hk);
            end
        end
    end

    numTracks = numel(tracks);
    fprintf('\nTracks created: %d (simulation truth drones: %d)\n', numTracks, numDrones);

    loggers = cellfun(@(tr) tr.logger, tracks, 'UniformOutput', false);
    [rmseOpt, trIdx, drIdx] = MetricsEvaluator.optimalTrackDroneRmse(loggers, dronePos);
    fprintf('\nOptimal track-to-drone RMSE (evaluation only) [m]:\n');
    for p = 1:numel(rmseOpt)
        fprintf('  Track %d -> drone %d: RMSE = %.2f\n', trIdx(p), drIdx(p), rmseOpt(p));
    end
    if ~isempty(rmseOpt)
        fprintf('  Overall mean RMSE (matched pairs): %.2f\n', mean(rmseOpt));
    end

    fprintf('\nPer-track NEES (vs matched drone when available):\n');
    for p = 1:numel(trIdx)
        ti = trIdx(p);
        di = drIdx(p);
        logg = tracks{ti}.logger;
        pt = dronePos{di};
        vtrue = cfg.droneInitVel(:, di);
        neesVals = nan(1, N);
        for kk = 1:N
            if all(isfinite(logg.x_hat(:, kk)))
                e = [pt(:, kk); vtrue] - logg.x_hat(:, kk);
                Pk = logg.P_hist(:, :, kk);
                if all(isfinite(Pk(:)))
                    neesVals(kk) = e' * (Pk \ e);
                end
            end
        end
        m = isfinite(neesVals);
        if any(m)
            fprintf('  Track %d (drone %d): mean NEES = %.2f\n', ti, di, mean(neesVals(m)));
        end
    end

    if numTracks > 0
        pTrueByTrack = repmat({nan(2, N)}, numTracks, 1);
        for p = 1:numel(trIdx)
            ti = trIdx(p);
            di = drIdx(p);
            if ti >= 1 && ti <= numTracks && di >= 1 && di <= numel(dronePos)
                pTrueByTrack{ti} = dronePos{di};
            end
        end

        measXY = nan(2, 0);
        for k = 1:N
            p_a = pA(:, k);
            n_in = CoordinateUtils.inwardNormal(p_a, center);
            tt = CoordinateUtils.tangentVector(p_a, center);
            R_radar_cols = [n_in, tt];
            for i = 1:numDrones
                if radarResults.detected(i, k)
                    rho = radarResults.rho_meas(i, k);
                    phi = radarResults.phi_meas(i, k);
                    if isfinite(rho) && isfinite(phi)
                        p_radar = [rho * cos(phi); rho * sin(phi)];
                        p_world = p_a + R_radar_cols * p_radar;
                        measXY(:, end+1) = p_world; %#ok<AGROW>
                    end
                end
            end
        end

        trackIds = cellfun(@(tr) tr.targetId, tracks);
        TrackingPlotter.plotAll(t, dronePos, loggers, trackIds, measXY);
        for j = 1:numTracks
            TrackingPlotter.plotTargetDetail(t, pTrueByTrack{j}, tracks{j}.logger, tracks{j}.targetId);
        end
    else
        fprintf('No tracks to plot.\n');
    end
    if usejava('desktop')
        drawnow;
        shg;
    end

    % Save all figures and append to markdown report
    reportPath = fullfile(projectRoot, 'docs', 'stage2_stage3_plots.md');
    meta = [
        "Script: scripts/run_stage3_tracking.m"
        "rng seed: 42"
        "numDrones: " + string(cfg.numDrones)
        "dt: " + string(cfg.dt)
        "Tracks created: " + string(numTracks)
    ];
    [pngRelPaths, ~] = FigureSaver.saveAllOpen(string(projectRoot), "stage3");
    FigureSaver.appendToReport(string(reportPath), "Stage 3 (tracking)", pngRelPaths, meta);

    % Export a stable subset for README/docs
    FigureSaver.exportSelected(string(projectRoot), "stage3", [
        "Stage 3: Tracking"
    ]);

    fprintf('\nStage 3 complete.\n');
end
