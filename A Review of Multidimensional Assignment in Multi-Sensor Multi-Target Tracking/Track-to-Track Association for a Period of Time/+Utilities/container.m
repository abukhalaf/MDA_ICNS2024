classdef container < handle & Utilities.utilities
    properties (GetAccess = public, SetAccess = private)
        localTracks
        reports
        assignments
        transitivityAssignments
    end
    methods (Access = public)
        function this = container(scene)
            arguments
                scene (1, 1) trackingScenario
            end
            this@Utilities.utilities(scene);
            this.localTracks = cell(this.totalTimeStep, this.sensorHolderPlatformNumber);
            this.reports = cell(this.totalTimeStep, this.sensorHolderPlatformNumber);
            this.assignments = cell(this.totalTimeStep, 1);
            this.transitivityAssignments = cell(this.totalTimeStep, 2);
        end
        function collect_local_tracks(this, localTracks, timeIndex)
            arguments
                this (1, 1) Utilities.container
                localTracks (:, 1) cell
                timeIndex (1, 1) {mustBeInteger, mustBePositive}
            end
            this.localTracks(timeIndex, :) = localTracks';
        end
        function collect_reports(this, reports, timeIndex)
            arguments
                this (1, 1) Utilities.container
                reports (:, 1) cell
                timeIndex (1, 1) {mustBeInteger, mustBePositive}
            end
            this.reports(timeIndex, :) = reports';
        end
        function collect_assignments(this, assignments, timeIndex)
            arguments
                this (1, 1) Utilities.container
                assignments {mustBeInteger, mustBeNonnegative}
                timeIndex (1, 1) {mustBeInteger, mustBePositive}
            end
            this.assignments{timeIndex} = assignments;
        end
        function collect_transitivity_assignments(this, assignments, indices, timeIndex)
            arguments
                this (1, 1) Utilities.container
                assignments (:, 1) cell
                indices (:, 2) {mustBeInteger, mustBePositive}
                timeIndex (1, 1) {mustBeInteger, mustBePositive}
            end
            this.transitivityAssignments{timeIndex, 1} = assignments;
            this.transitivityAssignments{timeIndex, 2} = indices;
        end
    end
end