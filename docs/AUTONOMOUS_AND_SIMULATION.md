# Autonomous and simulation

## One routine project

FRC and FTC execute the same versioned routine model produced by the Analytics visual editor or the
shared Kotlin DSL. A routine owns drive goals, waits, conditions, parallel/deadline groups, calls,
branches, and robot actions; there is no separate path file to select or keep in sync. The routine
is trigger-neutral, so it can also be a teleop macro or be called by another routine.

Canonical source files are checked into this repository:

```text
.ares/project.json
.ares/action-catalog.json
.ares/autonomous-catalog.json
.ares/routines/<id>.aresroutine
.ares/controllers/<id>.arescontroller
.ares/controls/<id>.arescontrols
```

`project.json` defines the league, coordinate convention, canonical field dimensions, and bumper
footprint used by both Analytics placement constraints and robot-side preflight.

The autonomous catalog supplies the starting pose, enabled state, display order, alliance policy,
and safe default separately from the reusable routine. FRC publishes enabled entry IDs to
`SmartDashboard/AvailableAutos` and reads `SmartDashboard/SelectedAuto` once in
`autonomousInit`. A missing or disabled request falls back deterministically to the configured
default, then `do-nothing`, then the first enabled entry.

Analytics automatically loads `.ares/action-catalog.json` while the robot is offline. The catalog
is authoritative; it does not discover actions by scraping Kotlin text. `FrcNativeAutoContractTest`
requires the generated keys and runtime capability factories to agree.

After editing the project, regenerate and verify the Kotlin compiled onto the RoboRIO:

```powershell
.\gradlew.bat generateAresProject
.\gradlew.bat verifyAresProject
```

`src/main/kotlin/com/areslib/frc/generated/GeneratedAresProject.kt` is checked in and must not be
edited by hand. `compileKotlin` depends on `verifyAresProject`, so stale generated code blocks a
build. Generation requires neither the RoboRIO nor a network connection.

## Coordinate and preflight contract

Autos are authored in Blue-alliance, corner-origin field coordinates:

- X increases from the Blue wall toward the Red wall.
- Y increases left when viewed from the Blue wall.
- Heading is radians internally and counter-clockwise positive.
- Red execution reflects X across the alliance wall, preserves Y, and maps heading to `pi - heading`.

Before motion, `FRCAutoOrchestrator` performs the following fail-closed preflight:

1. Resolve the selected entry from the generated, enabled autonomous catalog.
2. Require its referenced generated routine to exist.
3. Check the starting pose and every recursively called drive goal against the field using Marvin's current 0.80 m square bumper
   footprint.
4. Configure the generated action, condition, and drive factories.
5. Seed dyn4j, CTRE odometry, and Redux pose from the alliance-adjusted starting pose.
6. Request the routine through the shared deterministic `RoutineManager`.

Any failure before or during execution cancels the task tree, zeros drive and season targets,
invokes hardware safety, publishes `ARES/Auto/Error`, and latches the run blocked.

Legacy `.aresauto`, PathPlanner, and Choreo files are import compatibility only. New routines are
compiled into the robot program and are not loaded as loose deploy files at runtime.

## Generated controller bindings

Controller profiles store stable logical names plus independent `DESKTOP_GLFW`, `FTC`, and `FRC`
raw mappings. Control schemes can bind press/release/hold/repeat, chords, analog values,
thresholds, and hysteretic zones to catalog actions or routines.

The FRC binding host samples `GenericHID` raw axes and buttons into preallocated input frames, which
preserves Flydigi Vader 5 Pro extras that WPILib Driver Station actually exposes. `ARESRobot` has a
single-owner lifecycle hook for installing that host and then uses it instead of the hardcoded
teleop controller. The current generated project declares no complete control scheme, so robot
initialization does not install a host yet and hardcoded teleop remains authoritative. When a
scheme is added, its generated runtimes must be assembled and explicitly installed during robot
initialization; never run both owners together. Verify the FRC mapping on the Driver Station because
desktop GLFW raw indexes are not interchangeable.

A macro is just a reusable routine assigned to a controller event. Routine policies support ignore,
restart, queue, parallel, and toggle/cancel behavior, and mode transitions cancel active bindings.

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

Choose a generated entry by setting `SmartDashboard/SelectedAuto`; the repository includes
`sim-drive-and-shoot`. The simulator runs the production compiler, task executor, Redux actions,
swerve follower, and mechanism IO against a deterministic dyn4j world.

Before deployment:

- Run the selected auto for Blue and Red and verify the mirrored starting pose.
- Confirm every editor action appears in `.ares/action-catalog.json` and has a runtime factory.
- Exercise both successful and timed-out `shooter.feedWhenReady` behavior.
- Move a pose to a field edge and confirm preflight blocks a footprint that crosses the wall.
- Select a missing entry and confirm the safe fallback is reported and outputs remain safe.
- Run `verifyAresProject` and inspect the generated Kotlin diff.
- Run `..\verify-autos.ps1` from the workspace root.

The default simulator has no trusted feeder beam-break sensor, matching the current physical robot.
Do not force detector validity merely to make inventory bookkeeping advance.
