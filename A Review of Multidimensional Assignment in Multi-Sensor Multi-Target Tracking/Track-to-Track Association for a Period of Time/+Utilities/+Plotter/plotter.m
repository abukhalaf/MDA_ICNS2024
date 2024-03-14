classdef plotter < handle & Utilities.utilities
    properties (GetAccess = public, SetAccess = private)
        theater
        coverage
        sensorHolderPlatform
        targetPlatform
        trajectory
        detections
        localTracks
    end
    properties (GetAccess = public, Constant = true)
        colorPalette = [    0, 0.447, 0.741;
                         0.85, 0.325, 0.098;
                         0.25,   0.8,  0.54;
                        0.494, 0.184, 0.556;
                        0.466, 0.674, 0.188;
                        0.301, 0.745, 0.933;
                        0.635, 0.078, 0.184;
                            1,     0,     1;
                            1,  0.54,     0;
                        0.929, 0.694, 0.125];
    end
    methods (Access = public)
        function this = plotter(scene, options)
            arguments
                scene (1, 1) trackingScenario
                options.isDetectionPlotter (1, 1) logical = false;
                options.isLocalTrackPlotter (1, 1) logical = false;
            end
            this@Utilities.utilities(scene);
            % Surveillance Area Limits
            [XLim, YLim, ZLim] = this.surveillance_area_limits(scene);
            this.theater = theaterPlot("XLimits", XLim, ...
                                       "YLimits", YLim, ...
                                       "ZLimits", ZLim);
            % Theater Plotter Object
            this.coverage = coveragePlotter(this.theater, ...
                                            "DisplayName", "Sensor Coverage", ...
                                            "Alpha", [0.1, 0]);
            % Sensor Holder Platform Plotter Object
            this.sensorHolderPlatform = platformPlotter(this.theater, ...
                                                        "DisplayName", "Sensor Holder Platforms", ...
                                                        "Marker", "^", ...
                                                        "MarkerEdgeColor", "blue", ...
                                                        "MarkerFaceColor", "blue", ...
                                                        "MarkerSize", 6);
            % Target Platform Object
            this.targetPlatform = platformPlotter(this.theater, ...
                                                  "DisplayName", "Target Platforms", ...
                                                  "Marker", "^", ...
                                                  "MarkerEdgeColor", "black", ...
                                                  "MarkerFaceColor", "black", ...
                                                  "MarkerSize", 6);
            % Target True Trajectory Plotter Object
            colorIndex = 1;
            trajectoryPlotters = cell(this.targetPlatformNumber, 1);
            for i = 1:this.targetPlatformNumber
                if colorIndex > size(this.colorPalette, 1)
                    colorIndex = 1;
                end
                trajectoryPlotters{i} = trajectoryPlotter(this.theater, "DisplayName", "Target_{" + string(i) + "} Trajectory", ...
                                                                        "Color", this.colorPalette(colorIndex, :), ...
                                                                        "LineStyle", "--", ...
                                                                        "LineWidth", 1.2);
                colorIndex = colorIndex + 1;
            end
            this.trajectory = trajectoryPlotters;
            % Detections Plotter Object
            if options.isDetectionPlotter
                colorIndex = 1;
                detectionPlotters = cell(this.targetPlatformNumber, 1);
                for i = 1:this.targetPlatformNumber
                    if colorIndex > size(this.colorPalette, 1)
                        colorIndex = 1;
                    end
                    detectionPlotters{i} = detectionPlotter(this.theater, "DisplayName", "Target_{" + string(i) +  "} Detections", ...
                                                                          "Marker", "o", ...
                                                                          "MarkerSize", 6, ...
                                                                          "MarkerEdgeColor", "black", ...
                                                                          "MarkerFaceColor", this.colorPalette(colorIndex, :));
                    colorIndex = colorIndex + 1;
                end
                this.detections = detectionPlotters;
            end
            % Local Tracks Plotter Object
            if options.isLocalTrackPlotter
                localTrackPlotters = cell(this.sensorHolderPlatformNumber, this.targetPlatformNumber);
                for i = 1:this.sensorHolderPlatformNumber
                    colorIndex = 1;
                    for j = 1:this.targetPlatformNumber
                        if colorIndex > size(this.colorPalette, 1)
                            colorIndex = 1;
                        end
                        localTrackPlotters{i, j} = trackPlotter(this.theater, "DisplayName", "Local Track_{" + string(i) + "," + string(j) + "}", ...
                                                                              "HistoryDepth", 0, ...
                                                                              "ConnectHistory", "off", ...
                                                                              "ColorizeHistory", "off", ...
                                                                              "Marker", "s", ...
                                                                              "MarkerSize", 6, ...
                                                                              "MarkerEdgeColor", "black", ...
                                                                              "MarkerFaceColor", this.colorPalette(colorIndex, :));
                        colorIndex = colorIndex + 1;
                    end
                end
                this.localTracks = localTrackPlotters;
            end
        end
        function visualize_environment(this, scene, poses)
            arguments
                this (1, 1) Utilities.Plotter.plotter
                scene (1, 1) trackingScenario
                poses struct
            end
            plotPlatform(this.sensorHolderPlatform, vertcat(poses(1:this.sensorHolderPlatformNumber, 1).Position));
            plotPlatform(this.targetPlatform, vertcat(poses((this.sensorHolderPlatformNumber + 1):end, 1).Position));
            plotCoverage(this.coverage, coverageConfig(scene));
            for i = 1:this.targetPlatformNumber
                plotTrajectory(this.trajectory{i}, {vertcat(poses(i + this.sensorHolderPlatformNumber, :).Position)});
            end
            grid on
            view(70, 30);
        end
        function iterate_environment(this, poses, localTracks, reports, sensors)
            arguments
                this (1, 1) Utilities.Plotter.plotter
                poses(:, 1) struct
                localTracks (:, 1) cell
                reports (:, 1) cell
                sensors (:, 1) cell
            end
            this.update_platform_positions(poses);
            if ~isempty(this.detections)
                if reports{1}.NumReports > 0
                    this.update_detections(reports, sensors);
                end
            end
            if ~isempty(this.localTracks)
                this.update_local_tracks(localTracks);
            end
        end
    end
    methods (Access = private)
        function update_platform_positions(this, poses)
            arguments
                this (1, 1) Utilities.Plotter.plotter
                poses (:, 1) struct
            end
            plotPlatform(this.sensorHolderPlatform, vertcat(poses(1:this.sensorHolderPlatformNumber).Position));
            plotPlatform(this.targetPlatform, vertcat(poses((this.sensorHolderPlatformNumber + 1):end).Position));
        end
        function update_detections(this, reports, sensors)
            arguments
                this (1, 1) Utilities.Plotter.plotter
                reports (:, 1) cell
                sensors (:, 1) cell
            end
            reportsPerTarget = zeros(this.sensorHolderPlatformNumber, 3, this.targetPlatformNumber);
            for i = 1:this.sensorHolderPlatformNumber
                reps = reports{i}.Reports;
                objAttr = vertcat(reps.ObjectAttributes);
                objAttr = vertcat(objAttr{:});
                tgtInd = vertcat(objAttr.TargetIndex) - this.sensorHolderPlatformNumber;
                repsInCart = this.sph_to_cart(horzcat(reports{i}.Reports.Measurement)', sensors{i}.MountingAngles, sensors{i}.MountingLocation);
                for j = 1:this.targetPlatformNumber
                    ind = find(tgtInd == j);
                    reportsPerTarget(i, :, j) = repsInCart(ind, :);
                end
            end
            for i = 1:this.targetPlatformNumber
                plotDetection(this.detections{i}, reportsPerTarget(:, :, i));
            end
        end
        function update_local_tracks(this, tracks)
            arguments
                this (1, 1) Utilities.Plotter.plotter
                tracks cell
            end
            for i = 1:numel(tracks)
                for j = 1:numel(tracks{i})
                    trc = tracks{i}(j);
                    state = trc.State';
                    plotTrack(this.localTracks{i, trc.TrackID}, state(1:2:6), state(2:2:6));
                end
            end
        end
    end
    methods (Access = private, Static = true)
        function [XLim, YLim, ZLim] = surveillance_area_limits(scene)
            arguments
                scene (1, 1) trackingScenario
            end
            xEpansionCoeff = 1000;
            yEpansionCoeff = 5000;
            zEpansionCoeff = 1000;

            platformNum = numel(scene.Platforms);
            platformWaypoints = cell(platformNum, 1);
            [platformWaypoints{:}] = vertcat(scene.Platforms{:}).Trajectory;
            waypoints = [];
            for i = 1:platformNum
                waypoints = vertcat(waypoints, platformWaypoints{i}.Waypoints);
            end
            lowerLims = min(waypoints);
            upperLims = max(waypoints);
            XLim = [lowerLims(1) - xEpansionCoeff, upperLims(1) + xEpansionCoeff];
            YLim = [lowerLims(2) - yEpansionCoeff, upperLims(2) + yEpansionCoeff];
            ZLim = [lowerLims(3) - zEpansionCoeff, upperLims(3) + zEpansionCoeff];
        end
    end
end