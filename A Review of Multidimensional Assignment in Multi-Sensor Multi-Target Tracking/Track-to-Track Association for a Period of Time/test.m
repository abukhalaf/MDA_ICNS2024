clc; clear; close all;
rng(123456);
%% Choose Scenario
[file, path] = uigetfile("*.mat", "MultiSelect", "off");
file = fullfile(path, file);
load(file);
%% Ground Truth
poses = [groundTruth(:).Poses];
simTime = [groundTruth(:).SimulationTime];
%% Plotter
plt = Utilities.Plotter.plotter(scene, "isDetectionPlotter", true, "isLocalTrackPlotter", true);
plt.visualize_environment(scene, poses);
%% Local Tracker Object
HasLocalTrackUncertainty = false;
isAdditiveNose = true;
noiseLevel = [0.001, 0.001, 1];
lclTracker = LocalTracker.local_tracker(scene, poses(:, 1), "HasUncertainty", HasLocalTrackUncertainty, ...
                                                            "isAdditiveNoise", isAdditiveNose, ...
                                                            "noiseVariances", noiseLevel);
%% Global Fuser Object - 2D
fuser2d = GlobalFuser.global_fuser(scene, "Dimension", "2D", "Uncertainty", HasLocalTrackUncertainty);
%% Global Fuser Object - SD
fuserSd = GlobalFuser.global_fuser(scene, "Dimension", "SD", "Uncertainty", HasLocalTrackUncertainty);
%% Container - 2D
cont2d = Utilities.container(scene);
%% Container - SD
contSd = Utilities.container(scene);
%% Run Simulation
timeIndex = 1;
for time = simTime
    % Local Tracks
    [localTracks, reports] = lclTracker.step(sensors, poses(:, timeIndex), time);
    % Global Fuser - 2D
    [assignments2d, allTransAssn2d, transCombinations2d, seqCombinations2d, cost2D] = fuser2d.step(localTracks);
    % Global Fuser - SD
    [assignmentsSd, costSD] = fuserSd.step(localTracks);
    % Container - 2D
    cont2d.collect_reports(reports, timeIndex);
    cont2d.collect_local_tracks(localTracks, timeIndex);
    cont2d.collect_assignments(assignments2d, timeIndex);
    cont2d.collect_transitivity_assignments(allTransAssn2d, transCombinations2d, timeIndex);
    contSd.collect_assignments(assignmentsSd, timeIndex);
    % Plot Environment
    plt.iterate_environment(poses(:, timeIndex), localTracks, reports, sensors);
    timeIndex = timeIndex + 1;
end
%% Info
Utilities.info(lclTracker.sensorHolderPlatformNumber, lclTracker.targetPlatformNumber, lclTracker.distanceBetweenTargets);
%% Analysis
an = Utilities.analysis(scene);
[trueAssn2D, faultyAssn2D, transitivityError2D, trueAssn2DListed] = an.count_true_assignments_2D(cont2d.assignments, cont2d.transitivityAssignments, cont2d.localTracks);
[trueAssnSD, faultyAssnSD, trueAssnSDListed] = an.count_true_assignments_SD(contSd.assignments, cont2d.localTracks);

table(trueAssn2D, faultyAssn2D, transitivityError2D)
table(trueAssnSD, faultyAssnSD)
