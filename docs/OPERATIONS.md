# Build, test, deploy, and troubleshoot

## Prerequisites

- Windows PowerShell for the documented wrapper commands.
- WPILib 2026.2.1, including its Java 17 runtime and desktop native libraries.
- The sibling `../ARESLib-Kotlin` checkout. `settings.gradle` includes it as a composite build.
- Vendor dependencies installed/resolvable for CTRE Phoenix 6 and WPILib.
- An `scp` client and RoboRIO network access when running `fetchOffsets`.

The project uses Kotlin 1.9.23 and targets Java 17. Do not run robot builds with an arbitrary newer JVM when diagnosing native test or simulation issues; prefer the WPILib-provided Java runtime.

## Common commands

Run from the ARES-FRC repository root:

```powershell
# Compile and execute all JUnit 5 tests
.\gradlew.bat test

# Build robot artifacts
.\gradlew.bat build

# Launch WPILib desktop simulation
.\gradlew.bat simulateJava

# Deploy code and src/main/deploy contents to team 23247
.\gradlew.bat deploy -PteamNumber=23247

# Fetch the current runtime swerve calibration from the RoboRIO
.\gradlew.bat fetchOffsets
```

The Gradle deployment default is team `9999`, so always supply `-PteamNumber=23247` unless intentionally targeting another robot.

Tests use JUnit 5 and configure WPILib desktop JNI extraction. On Windows the build prefers `C:/Users/Public/wpilib/2026/jdk/bin/java.exe` when it exists.

If shared ARESLib code changes, its composite build is used by ARES-FRC. Other projects consume Maven snapshots, so publish ARESLib before testing those consumers:

```powershell
cd ..\ARESLib-Kotlin
.\gradlew.bat publishToMavenLocal
```

## Deployment checklist

1. Run `test` and a desktop simulation of the intended autonomous path.
2. Confirm `SmartDashboard/SelectedPath` is not the real-robot default `SimPath`.
3. Place competition `.path` files under `src/main/deploy/pathplanner/paths/`.
4. Verify `src/main/deploy/swerve_offsets.json` matches the robot.
5. Verify the cowl and climber encoder zero references before using position control.
6. Confirm both Limelights are reachable as `limelight-shooter` and `limelight-back`.
7. Deploy with the explicit team number.
8. While disabled, confirm alliance, pose, mechanism validity telemetry, and zero/safe outputs.
9. Enable mechanisms individually before running a full autonomous routine.

GradleRIO copies `src/main/deploy` to `/home/lvuser/deploy`. Files under `src/main/resources/deploy` are classpath resources for tests/simulation and are not a substitute for the RoboRIO deploy directory.

## Swerve offsets

The checked-in baseline is `src/main/deploy/swerve_offsets.json`, with keys:

- `frontLeft`
- `frontRight`
- `backLeft`
- `backRight`

The runtime robot may write `/home/lvuser/swerve_offsets_runtime.json`. The `fetchOffsets` task copies that file over the checked-in deploy JSON from `lvuser@10.232.47.2`.

Before the first fetch, connect once with `ssh lvuser@10.232.47.2 true` and verify the RoboRIO
fingerprint. The task requires a trusted `known_hosts` entry and fails on unknown or changed host
keys.

Treat `fetchOffsets` as a calibration update:

1. Put the robot in a mechanically known calibration state.
2. Generate/verify runtime offsets on the correct RoboRIO.
3. Run `fetchOffsets`.
4. Review the JSON diff for all four modules.
5. Rebuild and re-test steering orientation before deployment.

Do not fetch offsets from an unknown robot or network target and immediately deploy them.

## Troubleshooting

### Autonomous immediately stops

Check Driver Station errors and `SmartDashboard/SelectedPath` first. On a real robot, the default `SimPath` deliberately produces an empty path and fail-safe stop. Also verify the named file exists under `/home/lvuser/deploy/pathplanner/paths/` with the `.path` extension.

### A path works in tests but is missing on the RoboRIO

It is probably in `src/main/resources/deploy`, which is available on the test/simulation classpath but is not copied as a deploy asset. Move/copy the competition path to `src/main/deploy/pathplanner/paths/`.

### `FeederShoot` waits and then continues without a shot

The wait is intentionally capped at 2 s. Inspect flywheel target RPM, measured RPM, and `velocityValid`. A plausible cached RPM with invalid status is not ready. Readiness also requires a target above 100 RPM and less than 150 RPM error.

### Feeder/game-piece state never changes

Marvin XIX has no configured physical beam break, so `pieceDetectionValid` is false by design. The default desktop simulation mirrors this. Do not force validity true just to make inventory move; enable a real detector implementation or explicitly configure one in a test/simulation.

### Cowl moves far beyond the intended angle

The cowl API and shot table use mechanism **rotations**, not degrees. `0.50` means half a rotation. Valid commands are clamped to `0.0..1.80` rotations. Audit any dashboard or autonomous value that labels the field as degrees.

### Climber position is wrong or hits a soft limit

Position commands are mechanism rotations with an 80:1 sensor-to-mechanism ratio and hardware limits of `0.0..1.73` rotations. The code does not home the climber. Check encoder zero, calibration, and units; do not compensate by bypassing the soft limit.

### Robot drives 180 degrees from the expected field direction

Confirm the alliance value and coordinate convention. The pose frame is always blue-origin and CCW-positive. Teleop currently negates both translation axes on Red to preserve driver-forward perspective. Autonomous mirrors the path separately. Do not add an extra translation or heading inversion in swerve IO.

### Mechanism values freeze or jump to zero

Check the corresponding CTRE refresh status/validity before tuning gains. Hardware observations are cached once per loop. The flywheel reducer deliberately exposes zero RPM after an invalid refresh so stale data cannot authorize shooting.

### Any periodic exception or outputs unexpectedly zero

Driver Station should contain the originating exception. The robot intentionally calls `safeHardware()` on periodic failures, so zero outputs are evidence of the fail-safe path rather than a second fault. Fix the original exception before bypassing any safety call.

### Desktop tests fail to load WPILib/CTRE native libraries

Verify the WPILib 2026 installation and its Java 17 runtime at `C:/Users/Public/wpilib/2026/jdk/bin/java.exe`. Then rerun with the Gradle wrapper so the configured desktop JNI extraction is applied. Avoid launching test classes directly from an IDE until its native-library configuration matches Gradle.

### Changes in ARESLib are not visible elsewhere

ARES-FRC uses a composite sibling build, but ARES-FTC and ARES-Analytics may consume Maven/JitPack-style snapshots. Publish ARESLib to Maven Local, then rebuild the consumer. Do not copy shared classes into the season repository.
