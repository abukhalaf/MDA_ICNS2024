%% Clear workspace
clc; clear; close all;
%% Description
% ScenarioGenerator.m file creates the scenario environment
% according to the specified parameters
% 
% Things to be considered:
%     - Targets are set to follow line trajectory
%     - Targets move at a constant velocity throughout the scenario
%     - Each sensor is mounted on a unique sensor holder platform
%       This means that each sensor holder platform carries only one sensor
%     - Each sensor holder platform is equally spaced
%       on a circle with centre [0, 0, 0]
%% Environment Parameters
sensorNum = 2;
targetNum = 2;

distanceBetweenTargets = 20;        % [meter]
sensorHolderPlatformNum = sensorNum;

targetInitXpos = 15000;             % [meter]
targetVelocity = 0.25;              % [mach]
%% Simulation Prameters
simStopTime = 60;       % [sec]
simUpdateRate = 100;    % [hz]
%% Sensor Parameters
rdrUpdateRate = 10;     % [hz]
%% Scenario Object
scene = trackingScenario("IsEarthCentered", false, ...
                         "InitialAdvance", "Zero", ...
                         "StopTime", simStopTime, ...
                         "UpdateRate", simUpdateRate);
%% Sensor Objects (Identical sensors)
% Radar Cross Section
rcs = rcsSignature("Pattern", 0 * ones(2, 2));    % 1 [m^2] Radar Cross Section which is similar to F-16.
% Sensors
sensors = cell(sensorNum, 1);
for i = 1:sensorNum
    sensors{i} = fusionRadarSensor(i, "No scanning", ...
                                   "UpdateRate", rdrUpdateRate, ...
                                   "MountingLocation", [0, 0, 0], ...
                                   "MountingAngles", [0, 0, 0], ...
                                   "DetectionMode", "Monostatic", ...
                                   "HasElevation", true, ...
                                   "HasRangeRate", false, ...
                                   "HasNoise", true, ...
                                   "HasFalseAlarms", false, ...
                                   "HasOcclusion", true, ...
                                   "HasRangeAmbiguities", false, ...
                                   "HasINS", true, ...
                                   "MaxNumReportsSource", "Auto", ...
                                   "TargetReportFormat", "Clustered detections", ...
                                   "DetectionCoordinates", "Sensor spherical", ...
                                   "AzimuthResolution", 0.001, ...
                                   "ElevationResolution", 0.001, ...
                                   "RangeResolution", 10, ...
                                   "AzimuthBiasFraction", 0.1, ...
                                   "ElevationBiasFraction", 0.1, ...
                                   "RangeBiasFraction", 0.05, ...
                                   "CenterFrequency", 300e6, ...
                                   "Bandwidth", 3e6, ...
                                   "Sensitivity", -50, ...
                                   "DetectionProbability", 0.9, ...
                                   "ReferenceRange", 20e3, ...
                                   "ReferenceRCS", 0, ...
                                   "FalseAlarmRate", 1e-6, ...
                                   "FieldOfView", [30, 5], ...
                                   "RangeLimits", [0, 20e3], ...
                                   "InterferenceInputPort", false, ...
                                   "EmissionsInputPort", false);
    sensors{i}.Profiles.Signatures{1} = rcs;
end
%% Trajectories
% Sensor Holder Platform Positions (Stationary Platforms)
sensorHolderWaypoints = cell(sensorHolderPlatformNum, 1);
sensorHolderTraj = cell(sensorHolderPlatformNum, 1);
if sensorHolderPlatformNum == 1
    sensorHolderWaypoints{1} = [0, 0, 0;
                                0, 0, 0];
else
    % Circular Positions
    radius = 1000;           % [meter]
    theta = 0:pi / (sensorHolderPlatformNum - 1):pi;
    xPos = -radius * sin(theta)';
    yPos = radius * cos(theta)';
    for i = 1:sensorHolderPlatformNum
        sensorHolderWaypoints{i} = [xPos(i), yPos(i), 0;
                                    xPos(i), yPos(i), 0];
    end
end
for i = 1:sensorHolderPlatformNum
    sensorHolderTraj{i} = waypointTrajectory("Waypoints", sensorHolderWaypoints{i}, ...
                                             "TimeOfArrival", [0, simStopTime]);
end
% Target Platform Trajectories (Line Trajectory, Constant Velocity)
targetWaypoints = cell(targetNum, 1);
targetTraj = cell(targetNum, 1);
speedOfSound = 343;             % [meter / sec]
coeff = floor(targetNum / 2);
if targetNum == 1
    targetWaypoints{1} = [                                                targetInitXpos, 0, 0;
                          targetInitXpos - (targetVelocity * speedOfSound * simStopTime), 0, 0];
end
for i = 1:targetNum
    if mod(targetNum, 2) == 0
        targetYpos = distanceBetweenTargets * ((i - 1) - coeff) + (distanceBetweenTargets / 2);
    else
        targetYpos = distanceBetweenTargets * ((i - 1) - coeff);
    end
    targetWaypoints{i} = [                                                targetInitXpos, targetYpos, 0;
                          targetInitXpos - (targetVelocity * speedOfSound * simStopTime), targetYpos, 0];
end
for i = 1:targetNum
    targetTraj{i} = waypointTrajectory("Waypoints", targetWaypoints{i}, ...
                                       "TimeOfArrival", [0, simStopTime], ...
                                       "AutoBank", true, ...
                                       "AutoPitch", true);
end
%% Platforms
platforms = cell(sensorHolderPlatformNum + targetNum, 1);
% Sensor Holder Platforms
for i = 1:sensorHolderPlatformNum
    platforms{i} = platform(scene, ...
                            "ClassID", 0, ...
                            "Trajectory", sensorHolderTraj{i}, ...
                            "Sensors", sensors(i), ...
                            "Signatures", {});
end
% Target Platforms
for i = 1:targetNum
    platforms{i + sensorHolderPlatformNum} = platform(scene, ...
                                                      "ClassID", 3, ...
                                                      "Trajectory", targetTraj{i}, ...
                                                      "Signatures", {rcs});
end
%% Ground Truth
groundTruth = record(scene);
%% Visualize Environment
poses = [groundTruth(:).Poses];
plt = Utilities.Plotter.plotter(scene);
plt.visualize_environment(scene, poses);
%% Save Simulation
path = "Datasets/";
if ~exist(path, "dir")
   mkdir(path);
end
sims = dir(path + "*.mat");
simNum = numel(sims) + 1;
simName = "dataset" + string(simNum);
save(path + simName, "scene", "groundTruth", "platforms", "sensors");