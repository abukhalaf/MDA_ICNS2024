classdef local_tracker < handle & Utilities.utilities
    properties (GetAccess = public, SetAccess = private)
        % Main Parameters
        filters
        tracks
        isInitialTimeStep

        % Optional Parameters
        hasUncertainty
        isAdditiveNoise
        noiseVariances
    end
    methods (Access = public)
        function this = local_tracker(scene, initialPoses, options)
            arguments
                scene (1, 1) trackingScenario
                initialPoses (:, 1) struct
                options.HasUncertainty (1, 1) logical = false;
                options.isAdditiveNoise (1, 1) logical = false;
                options.noiseVariances (1, 3) {mustBeReal, mustBeNonnegative} = [1e-3, 1e-3, 1];
            end
            this@Utilities.utilities(scene);
            this.hasUncertainty = options.HasUncertainty;
            this.isAdditiveNoise = options.isAdditiveNoise;
            this.noiseVariances = options.noiseVariances;
            this.isInitialTimeStep = true;

            targetPoses = initialPoses((this.sensorHolderPlatformNumber + 1):end);
            % Filter Initialization
            filts = cell(this.sensorHolderPlatformNumber, 1);
            for k = 1:this.sensorHolderPlatformNumber
                localFilters = [];
                for i = 1:this.targetPlatformNumber
                    states = zeros(6, 1);
                    localFilters = [localFilters; ...
                                    trackingEKF("State", states, ...
                                                "StateCovariance", eye(6, 6), ...
                                                "StateTransitionFcn", @constvel, ...
                                                "StateTransitionJacobianFcn", @constveljac, ...
                                                "ProcessNoise", 1, ...
                                                "HasAdditiveProcessNoise", true, ...
                                                "MeasurementFcn", @cvmeas, ...
                                                "MeasurementJacobianFcn", @cvmeasjac, ...
                                                "MeasurementNoise", 1, ...
                                                "HasAdditiveMeasurementNoise", true)];
                end
                filts{k} = localFilters;
            end
            this.filters = filts;

            % Track Initialization
            lclTrcks = cell(this.sensorHolderPlatformNumber, 1);
            for k = 1:this.sensorHolderPlatformNumber
                localTracks = [];
                for i = 1:this.targetPlatformNumber
                    localTracks = [localTracks;
                                   objectTrack("TrackID", i, ...
                                               "SourceIndex", k, ...
                                               "UpdateTime", 0, ...
                                               "State", filts{k}(i).State, ...
                                               "StateCovariance", filts{k}(i).StateCovariance, ...
                                               "ObjectAttributes", struct("TargetIndex", targetPoses(i).PlatformID))];
                end
                lclTrcks{k} = localTracks;
            end
            this.tracks = lclTrcks;
        end
        function [tracks, reports] = step(this, sensors, poses, time)
            arguments
                this (1, 1) LocalTracker.local_tracker
                sensors (:, 1) cell
                poses (:, 1) struct
                time (1, 1) {mustBeReal, mustBeNonnegative}
            end
            [reports, isValidTime] = this.get_reports(sensors, poses, time);
            if this.isInitialTimeStep
                this.initialize_filters(reports, sensors);
                this.isInitialTimeStep = false;
            end
            if isValidTime
                if this.isAdditiveNoise
                    reports = this.additive_noise(reports);
                end
                this.update_local_tracks(reports, sensors, time);
            else
                this.predict_local_tracks(time);
            end
            if this.hasUncertainty
                tracks = this.get_uncertain_local_tracks();
            else
                tracks = this.tracks;
            end
        end
    end
    methods (Access = private)
        function [allReports, isValidTime] = get_reports(this, sensors, poses, time)
            arguments
                this (1, 1) LocalTracker.local_tracker
                sensors (:, 1) cell
                poses struct
                time (1, 1) {mustBeReal, mustBeNonnegative}
            end
            sensorNum = this.sensorHolderPlatformNumber;
            sensorHolderPlatformPoses = poses(1:this.sensorHolderPlatformNumber);
            targetPoses = poses((this.sensorHolderPlatformNumber + 1):end);
            allReports = cell(sensorNum, 1);
            for i = 1:sensorNum
                [reports, numReports, config] = sensors{i}(targetPoses, struct("Position", sensorHolderPlatformPoses(i).Position, ...
                                                                               "Velocity", sensorHolderPlatformPoses(i).Velocity, ...
                                                                               "Orientation", sensorHolderPlatformPoses(i).Orientation), time);
                allReports{i} = struct("Reports", vertcat(reports{:}), "NumReports", numReports, "Config", config);
            end
            isValidTime = config.IsValidTime;
        end
        function update_local_tracks(this, reports, sensors, time)
            arguments
                this (1, 1) LocalTracker.local_tracker
                reports (:, 1) cell
                sensors (:, 1) cell
                time (1, 1) {mustBeReal, mustBeNonnegative}
            end
            for i = 1:this.sensorHolderPlatformNumber
                reps = reports{i}.Reports;
                reportObjectAttributes = vertcat(reps.ObjectAttributes);
                reportObjectAttributes = vertcat(reportObjectAttributes{:});
                reportTargetIndices = vertcat(reportObjectAttributes.TargetIndex);
                repsInCart = this.sph_to_cart(horzcat(reps.Measurement)', sensors{i}.MountingAngles, sensors{i}.MountingLocation);
                for j = 1:this.targetPlatformNumber
                    ind = reportTargetIndices(j) - this.sensorHolderPlatformNumber;
                    correct(this.filters{i}(ind), repsInCart(j, :));
                    this.tracks{i}(ind).State = this.filters{i}(ind).State;
                    this.tracks{i}(ind).StateCovariance = this.filters{i}(ind).StateCovariance;
                    this.tracks{i}(ind).UpdateTime = time;
                end
            end
        end
        function predict_local_tracks(this, time)
            arguments
                this (1, 1) LocalTracker.local_tracker
                time (1, 1) {mustBeReal, mustBeNonnegative}
            end
            for i = 1:this.sensorHolderPlatformNumber
                for j = 1:this.targetPlatformNumber
                    predict(this.filters{i}(j), this.samplingPeriod);
                    this.tracks{i}(j).State = this.filters{i}(j).State;
                    this.tracks{i}(j).StateCovariance = this.filters{i}(j).StateCovariance;
                    this.tracks{i}(j).UpdateTime = time;
                end
            end
        end
        function uncertainLocalTracks = get_uncertain_local_tracks(this)
            arguments
                this (1, 1) LocalTracker.local_tracker
            end
            uncertainLocalTracks = this.tracks;
            uncertainSensorNum = randi(numel(uncertainLocalTracks) + 1) - 1;
            whichSensors = randperm(numel(uncertainLocalTracks), uncertainSensorNum);
            for i = 1:uncertainSensorNum
                uncertainTargetNum = randi(numel(uncertainLocalTracks{i}) - 1) - 1;
                whichTargets = randperm(numel(uncertainLocalTracks{i}), uncertainTargetNum);
                uncertainLocalTracks{whichSensors(i)}(whichTargets) = [];
            end
        end
        function reports = additive_noise(this, reports)
            arguments
                this (1, 1) LocalTracker.local_tracker
                reports (:, 1) cell
            end
            covarianceMat = diag(this.noiseVariances);
            for k = 1:this.sensorHolderPlatformNumber
                noisyReports = reports{k}.Reports;
                noiseVec = mvnrnd([0, 0, 0], covarianceMat, numel(noisyReports));
                for i = 1:numel(noisyReports)
                    noisyReports(i).Measurement = noisyReports(i).Measurement + noiseVec(i, :)';
                    noisyReports(i).MeasurementNoise = noisyReports(i).MeasurementNoise + covarianceMat;
                end
                reports{k}.Reports = noisyReports;
            end
        end
        function initialize_filters(this, reports, sensors)
            arguments
                this (1, 1) LocalTracker.local_tracker
                reports (:, 1) cell
                sensors (:, 1) cell
            end
            for i = 1:this.sensorHolderPlatformNumber
                reps = reports{i}.Reports;
                objAttr = vertcat(reps.ObjectAttributes);
                objAttr = vertcat(objAttr{:});
                tgtInd = vertcat(objAttr.TargetIndex) - this.sensorHolderPlatformNumber;
                repsInCart = this.sph_to_cart(horzcat(reps.Measurement)', sensors{i}.MountingAngles, sensors{i}.MountingLocation);
                for j = 1:this.targetPlatformNumber
                    ind = find(tgtInd == j);
                    this.filters{i}(j).State(1:2:6) = repsInCart(ind, :);
                end
            end
        end
    end
end