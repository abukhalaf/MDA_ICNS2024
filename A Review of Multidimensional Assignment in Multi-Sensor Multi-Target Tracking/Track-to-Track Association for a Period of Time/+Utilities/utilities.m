classdef utilities < handle
    properties (GetAccess = public, SetAccess = protected)
        totalTimeStep
        samplingPeriod

        distanceBetweenTargets

        sensorHolderPlatformNumber
        targetPlatformNumber
    end
    methods (Access = public)
        function this = utilities(scene)
            arguments
                scene (1, 1) trackingScenario
            end
            [this.sensorHolderPlatformNumber, this.targetPlatformNumber] = this.get_platform_numbers(scene);
            this.totalTimeStep = scene.StopTime * scene.UpdateRate + 1;
            this.samplingPeriod = 1 / scene.UpdateRate;
            this.distanceBetweenTargets = norm(scene.Platforms{end}.Position - scene.Platforms{end - 1}.Position);
        end
    end
    methods (Access = protected, Static = true)
        function cart = sph_to_cart(sph, sensorAngle, sensorPosition)
            arguments
                sph (:, 3) {mustBeReal}
                sensorAngle (1, 3) {mustBeReal}
                sensorPosition (1, 3) {mustBeReal}
            end
            sphRad = [deg2rad(sph(:, 1)), deg2rad(sph(:, 2)), sph(:, 3)];
            [x, y, z] = sph2cart(sphRad(:, 1), sphRad(:, 2), sphRad(:, 3));
            sensorAngleQuat = quaternion(eul2quat(deg2rad(sensorAngle), "ZYX"));
            cart = rotateframe(sensorAngleQuat, [x, y, z]);
            cart = cart + sensorPosition;
        end
    end
    methods (Access = private, Static = true)
        function [sensorHolderPlatformNum, targetPlatformNum] = get_platform_numbers(scene)
            arguments
                scene (1, 1) trackingScenario
            end
            sensorHolderPlatformNum = 0;
            targetPlatformNum = 0;
            for i = 1:numel(scene.Platforms)
                if ~isempty(scene.Platforms{i}.Sensors)
                    sensorHolderPlatformNum = sensorHolderPlatformNum + 1;
                else
                    targetPlatformNum = targetPlatformNum + 1;
                end
            end
        end
    end
end