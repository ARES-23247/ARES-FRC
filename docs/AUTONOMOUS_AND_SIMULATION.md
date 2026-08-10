# Autonomous and simulation

## One native auto format

FRC and FTC execute the same versioned `.aresauto` document produced by the Analytics visual
editor and the shared Kotlin DSL. An auto owns its starting pose, drive goals, waits, parallel
groups, event markers, and robot actions; there is no separate FRC path file to select or keep in
sync.

Analytics stores FRC autos in `src/main/deploy/ares/autos`. GradleRIO copies that directory to
`/home/lvuser/deploy/ares/autos` on the RoboRIO. The selected document ID is read from
`SmartDashboard/SelectedAuto`; `do-nothing` is the safe default. `SmartDashboard/AvailableAutos`
contains the deploy-time catalog so dashboards can present a selector without scanning the robot
filesystem themselves.

The reserved `do-nothing` default compiles like every other document but deliberately does not reset
localization. This avoids replacing a valid disabled-period pose with an arbitrary fallback pose.

The editor's offline action menu comes from `src/main/deploy/ares/auto-capabilities.json`.
`FrcNativeAutoContractTest` requires that manifest, `FrcAutoCapabilities.descriptors`, and the
runtime `NamedCommands` registry remain identical.

## Coordinate and preflight contract

Autos are authored in Blue-alliance, corner-origin field coordinates:

- X increases from the Blue wall toward the Red wall.
- Y increases left when viewed from the Blue wall.
- Heading is radians internally and counter-clockwise positive.
- Red execution reflects X across the alliance wall, preserves Y, and maps heading to `pi - heading`.

Before motion, `FRCAutoOrchestrator` performs the following atomic preflight:

1. Resolve and decode the selected `.aresauto` without accepting arbitrary paths.
2. Require the filename and embedded document ID to agree.
3. Check every starting/goal pose against the field using Marvin's current 0.80 m square bumper
   footprint.
4. Validate every named action against the runtime capability registry.
5. Generate all swerve trajectories with the shared jerk-limited planner.
6. Seed dyn4j, CTRE odometry, and Redux pose from the alliance-adjusted starting pose.
7. Arm the shared deterministic `TaskExecutor`.

Any failure before or during execution cancels the task tree, zeros drive and season targets,
invokes hardware safety, publishes `ARES/Auto/Error`, and latches the run blocked.

## Available Marvin actions

| Action key | Behavior |
|---|---|
| `intake.collect` | Deploys intake and runs intake/floor rollers. |
| `intake.stop` | Stops intake/floor rollers without moving the pivot. |
| `intake.stow` | Stops rollers and retracts the pivot. |
| `shooter.prepare` | Commands the 4000 RPM autonomous preset. |
| `shooter.feedWhenReady` | Waits up to 2 s for a fresh RPM sample within 150 RPM, then runs feeder/floor. |
| `shooter.stop` | Clears flywheel, feeder, floor, and transfer targets. |

Place `shooter.feedWhenReady` in a drive goal's **On arrival** list when the chassis should stop and
wait before firing. A path marker is concurrent with drive motion and therefore should only be used
when firing while moving is intentional. A readiness timeout continues without firing and leaves
the feeder/floor safely stopped.

## Desktop simulation

Start WPILib desktop simulation with:

```powershell
.\gradlew.bat simulateJava
```

Choose a native document by setting `SmartDashboard/SelectedAuto`; the repository includes
`sim-drive-and-shoot`. The simulator runs the production compiler, task executor, Redux actions,
swerve follower, and mechanism IO against a deterministic dyn4j world.

Before deployment:

- Run the selected auto for Blue and Red and verify the mirrored starting pose.
- Confirm every editor action appears in the offline catalog and is recognized at preflight.
- Exercise both successful and timed-out `shooter.feedWhenReady` behavior.
- Move a pose to a field edge and confirm preflight blocks a footprint that crosses the wall.
- Select a missing document and confirm all outputs remain zero.
- Run `..\verify-autos.ps1` from the workspace root.

The default simulator has no trusted feeder beam-break sensor, matching the current physical robot.
Do not force detector validity merely to make inventory bookkeeping advance.
