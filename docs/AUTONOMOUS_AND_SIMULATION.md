# Autonomous and simulation

## Path selection and loading

`FRCAutoOrchestrator` reads `SmartDashboard/SelectedPath` at autonomous initialization. The default is `SimPath`.

On a real robot, leaving the selection at `SimPath` is treated as “no autonomous selected.” The orchestrator reports an error, creates an empty path, and the first autonomous loop enters the fail-safe stop. Select an intentional path name before enabling.

`PathLoader` resolves `<name>.path` in this order:

1. RoboRIO deploy directory: `pathplanner/paths/<name>.path`.
2. Classpath resource: `/deploy/pathplanner/paths/<name>.path`, used by tests and desktop simulation.

These locations are not interchangeable. Put competition paths in `src/main/deploy/pathplanner/paths/`; GradleRIO copies that directory to `/home/lvuser/deploy`. The repository's `src/main/resources/deploy/pathplanner/paths/SimPath.path` is a classpath fixture and is not a static RoboRIO deploy asset.

After loading, the path is mirrored for the active alliance using the canonical ARESLib FRC field dimensions. The first path point seeds all three pose owners:

- dyn4j ground truth in simulation,
- the physical CTRE swerve estimator when present,
- the Redux pose estimator through a reset `PoseUpdate`.

NetworkTables entries `Auto/InitialPoseX`, `Auto/InitialPoseY`, and `Auto/InitialPoseHeading` can override the path's initial pose. The heading override is radians and follows the CCW-positive field convention.

## Profile and tracking behavior

Path points contain cumulative distance and endpoint velocity. At initialization the orchestrator builds a time line for each segment:

- With usable endpoint velocity, segment time is `2 * distance / (startVelocity + endVelocity)`.
- Near a zero-velocity segment, a 0.10 m/s minimum profile velocity prevents an infinite duration.
- Within a segment, distance is evaluated with constant acceleration between endpoint velocities.

The time-based distance selects the target path sample. Separately, the robot's closest path distance is searched in a rolling +/-0.75 m window. This distinction is intentional:

- profile distance controls the target pose and feedforward;
- closest actual distance triggers event markers, so an event does not fire merely because time elapsed while the robot lagged behind.

The holonomic controller tracks target X, Y, heading, path tangent, and path velocity. At path end, feedforward velocity becomes zero and feedback continues to hold the final pose.

Loop `dt` is clamped to `0.005..0.100` seconds. A missing/empty path or periodic exception latches an autonomous fault and invokes the fail-safe stop.

## Supported event markers

Event names are case-sensitive and must match exactly:

| Event | Behavior |
|---|---|
| `FlywheelOn` | Commands the shooter to 4000 RPM. |
| `IntakeDeploy` | Deploys the intake and commands its roller to 15 RPS. |
| `FeederShoot` | Waits for a fresh, aligned flywheel observation, then begins shooting and commands the feeder to 10 RPS. |

Unknown marker names currently complete without an action. Treat that as a configuration error; validate spelling during simulation.

### `FeederShoot` wait semantics

The flywheel is ready only when its velocity refresh is valid, target speed is above 100 RPM, and absolute speed error is below 150 RPM.

If it is not ready at the marker:

1. Profile time and target distance are frozen at the marker.
2. Path velocity feedforward is set to zero while positional feedback holds the marker pose.
3. Readiness is checked again each loop.
4. After 2.0 seconds, the marker completes without firing and the path resumes.

If readiness becomes true before timeout, shooting begins, the event is marked triggered, and the profile resumes. Triggered marker indices are retained so the same event cannot run twice.

This is a bounded wait, not a guarantee that a shot occurred. Autonomous validation should inspect telemetry or state to distinguish successful firing from timeout.

## Desktop simulation

Start WPILib desktop simulation with:

```powershell
.\gradlew.bat simulateJava
```

`ARESRobot` uses simulated mechanism IO plus a seeded dyn4j world. The simulation models:

- swerve chassis motion and collisions,
- field boundaries and Crescendo field elements,
- intake capture and piece transfer,
- flywheel, cowl, feeder, floor, and climber state,
- 2.5D launched-piece flight, scoring, and re-entry,
- mechanism/field poses published for 3D visualization.

Simulation pose is dyn4j ground truth and is dispatched into the same Redux pose state consumed by teleop and autonomous code. This keeps the season controllers unchanged while replacing only the hardware boundary.

The physics world uses a deterministic seed (`42`) by default, which makes regression tests reproducible. Top-level `simgui*.json`, `networktables.json`, and `marvin19_layout.json` provide desktop visualization/dashboard configuration.

### Detector caveat

`SimulatedFeederIO` supports an optional piece detector, but the normal `ARESRobot` simulation constructs it disabled. Its detection validity is therefore false, matching the physical Marvin XIX feeder's lack of a beam break. Visual piece capture still exists in dyn4j, but Redux inventory transitions that require a trusted detector will not occur unless a test or custom simulation explicitly enables the detector.

### What to validate before deployment

- Run the selected path for both alliances and verify the mirrored initial pose.
- Confirm every event name is recognized and occurs at the intended actual path distance.
- Exercise the slow-spin case and observe both successful `FeederShoot` readiness and the 2 s timeout.
- Verify cowl values as mechanism rotations and flywheel values as RPM.
- Force a missing path and confirm drive/mechanism outputs go safe.
- Remember that simulated sensor validity is part of the contract; do not replace it with assumed-perfect data in production.
