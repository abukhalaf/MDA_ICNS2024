Dataset specifications are given in following table.

| Dataset Name  | Sensor Number | Target Number | Distance between Targets |
| :------------ | :-----------: | :-----------: | :----------------------: |
| dataset1.mat  | 2             | 3             | 250                      |
| dataset2.mat  | 2             | 3             | 100                      |
| dataset3.mat  | 2             | 3             | 50                       |
| dataset4.mat  | 2             | 3             | 20                       |
| dataset5.mat  | 2             | 5             | 250                      |
| dataset6.mat  | 2             | 5             | 100                      |
| dataset7.mat  | 2             | 5             | 50                       |
| dataset8.mat  | 2             | 5             | 20                       |
| dataset9.mat  | 2             | 10            | 250                      |
| dataset10.mat | 2             | 10            | 100                      |
| dataset11.mat | 2             | 10            | 50                       |
| dataset12.mat | 2             | 10            | 20                       |
| dataset13.mat | 3             | 3             | 250                      |
| dataset14.mat | 3             | 3             | 100                      |
| dataset15.mat | 3             | 3             | 50                       |
| dataset16.mat | 3             | 3             | 20                       |
| dataset17.mat | 3             | 5             | 250                      |
| dataset18.mat | 3             | 5             | 100                      |
| dataset19.mat | 3             | 5             | 50                       |
| dataset20.mat | 3             | 5             | 20                       |
| dataset21.mat | 3             | 10            | 250                      |
| dataset22.mat | 3             | 10            | 100                      |
| dataset23.mat | 3             | 10            | 50                       |
| dataset24.mat | 3             | 10            | 20                       |
| dataset25.mat | 4             | 3             | 250                      |
| dataset26.mat | 4             | 3             | 100                      |
| dataset27.mat | 4             | 3             | 50                       |
| dataset28.mat | 4             | 3             | 20                       |
| dataset29.mat | 4             | 5             | 250                      |
| dataset30.mat | 4             | 5             | 100                      |
| dataset31.mat | 4             | 5             | 50                       |
| dataset32.mat | 4             | 5             | 20                       |
| dataset33.mat | 4             | 10            | 250                      |
| dataset34.mat | 4             | 10            | 100                      |
| dataset35.mat | 4             | 10            | 50                       |
| dataset36.mat | 4             | 10            | 20                       |

All of the datasets are of the structure shown in the figure below for an example of a $2$-sensor $N$-target $250$ m separation distance.

![Simulation environment with $2$-sensor $N$-target $250$ m separation distance](https://github.com/abukhalaf/MDA_ICNS2024/blob/main/A%20Review%20of%20Multidimensional%20Assignment%20in%20Multi-Sensor%20Multi-Target%20Tracking/Figures/scenario_environment.jpg)

The sensors in datasets are designed to be synchronous RADARs with identical properties as below:

| Property              | Value                |
| :-------------------- | :------------------: |
| UpdateRate            | 10                   |
| MountingLocation      | [0, 0, 0]            |
| MountingAngles        | [0, 0, 0]            |
| DetectionMode         | Monostatic           |
| ScanMode              | No scanning          |
| HasElevation          | true                 |
| HasRangeRate          | false                |
| HasNoise              | true                 |
| HasFalseAlarms        | false                |
| HasOcclusion          | true                 |
| HasRangeAmbiguities   | false                |
| HasINS                | true                 |
| MaxNumReportsSource   | Auto                 |
| TargetReportFormat    | Clustered detections |
| DetectionCoordinates  | Sensor spherical     |
| AzimuthResolution     | 0.001                |
| ElevationResolution   | 0.001                |
| RangeResolution       | 10                   |
| AzimuthBiasFraction   | 0.1                  |
| ElevationBiasFraction | 0.1                  |
| RangeBiasFraction     | 0.05                 |
| CenterFrequency       | 300e6                |
| Bandwidth             | 3e6                  |
| Sensitivity           | -50                  |
| DetectionProbability  | 0.9                  |
| ReferenceRange        | 20e3                 |
| ReferenceRCS          | 0                    |
| FalseAlarmRate        | 1e-6                 |
| FieldOfView           | [30, 5]              |
| RangeLimits           | [0, 20e3]            |
| InterferenceInputPort | false                |
| EmissionsInputPort    | false                |

All targets are identical objects following a line-trajectory with a constant speed of $0.25$ Mach and a radar cross section (RCS) of $1$ $\text{m}^2$.

The [*trackingScenario*](https://www.mathworks.com/help/fusion/ref/trackingscenario.html) is used to set up the simulation environment, the [*fusionRadarSensor*](https://www.mathworks.com/help/fusion/ref/fusionradarsensor-system-object.html) is used to simulate RADAR sensors, and the [*platform*](https://www.mathworks.com/help/fusion/ref/platform.html) is used for the sensor holder or target aircraft platforms.
