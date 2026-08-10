# ARES-FRC Code Readability Audit

## Scope and accounting

Audit date: 2026-08-10.

The inventory was produced from every `.kt` and `.java` file under `src`, excluding build and
generated-output directories. It contains:

- 57 source/test files discovered.
- 56 first-party Kotlin files reviewed: 36 production files and 20 test files.
- 46 first-party files changed: all 36 production files and 10 test files.
- 10 first-party files reviewed without changes, all tests.
- 1 generated Java file audited conservatively and excluded from edits.

The pass removed 442 generated placeholder KDoc occurrences from first-party code. The only
remaining `Documentation for ...` placeholder under `src` belongs to the excluded generated file.
Comments and KDocs were then checked against current units, coordinate frames, control modes,
freshness/fail-closed behavior, WPILib lifecycle, Redux ownership, and simulation semantics.

No Gradle command was run, by coordination request. Validation was limited to static inspection,
placeholder searches, whitespace checks, and source-inventory reconciliation.

This audit file (`docs/CODE_READABILITY_AUDIT.md`) was created as the accounting artifact; it is
not included in the 56-file `src` inventory.

## Exclusion

- `src/main/java/frc/robot/generated/TunerConstants.java` — reviewed for integration assumptions,
  but not hand-edited because its Phoenix Tuner header marks it as generated code. Its placeholder
  class comment should be corrected in the generator/template, not in the generated artifact.

## Changed production files (36)

### Robot lifecycle and path ownership

- `src/main/kotlin/com/areslib/frc/ARESRobot.kt` — documented TimedRobot/Redux ownership, 20 ms
  ordering, simulation pose flow, and alliance mapping; removed unused imports and redundant
  nullable-style registration of non-null devices.
- `src/main/kotlin/com/areslib/frc/Dyn4jSimulation.kt` — documented field and projectile units,
  deterministic step ownership, the internal simulation bus, detector validity, pose reset, and
  visualization boundaries.
- `src/main/kotlin/com/areslib/frc/Main.kt` — replaced the generic template warning with the actual
  WPILib entry-point and initialization contract.
- `src/main/kotlin/com/areslib/frc/robot/FrcAutoCapabilities.kt` — defines the source-owned native
  auto action catalog, fresh task factories, and bounded flywheel-readiness gate.

### Hardware boundaries

- `src/main/kotlin/com/areslib/frc/hardware/FRCClimberHardwareIO.kt` — documented mechanism
  rotations, gearing, cached refresh ownership, effort scaling, and soft limits; removed unused
  Phoenix imports.
- `src/main/kotlin/com/areslib/frc/hardware/FRCCowlHardwareIO.kt` — made the rotations-not-degrees
  contract explicit and documented refresh/effort/limit invariants; removed unused imports.
- `src/main/kotlin/com/areslib/frc/hardware/FRCFeederHardwareIO.kt` — documented that the absent
  physical detector reports invalid and that `false` is not a trusted no-piece sample.
- `src/main/kotlin/com/areslib/frc/hardware/FRCFloorHardwareIO.kt` — documented voltage, rotations
  per second, and cached-signal boundaries; removed an unused import.
- `src/main/kotlin/com/areslib/frc/hardware/FRCFlywheelHardwareIO.kt` — documented RPM/RPS
  conversion, cached refresh, fail-closed velocity validity, and reverse lockout.
- `src/main/kotlin/com/areslib/frc/hardware/FRCIntakeHardwareIO.kt` — documented pivot degrees,
  mechanism rotations, roller RPS, and single-refresh read ownership; removed an unused import.
- `src/main/kotlin/com/areslib/frc/hardware/SeasonInterfaces.kt` — made the shared-interface aliases
  unambiguous by aliasing imports explicitly and noted that units/safe defaults live in ARESLib.
- `src/main/kotlin/com/areslib/frc/hardware/TalonFXExtensions.kt` — replaced a stale generated-style
  comment with the compatibility-source intent.

### Marvin state, reducers, and controllers

- `src/main/kotlin/com/areslib/frc/marvin/MarvinAction.kt` — documented action units, timestamps,
  freshness bits, cached sensor-snapshot ownership, and slamtake phase semantics.
- `src/main/kotlin/com/areslib/frc/marvin/MarvinClimberSubsystem.kt` — documented explicit voltage
  versus position modes and corrected motor-rotation wording to mechanism rotations.
- `src/main/kotlin/com/areslib/frc/marvin/MarvinConfig.kt` — documented every shot-table unit,
  rearward aim convention, cowl travel, and official blue-origin speaker coordinates.
- `src/main/kotlin/com/areslib/frc/marvin/MarvinControllerBase.kt` — documented shared Redux ownership.
- `src/main/kotlin/com/areslib/frc/marvin/MarvinCowlController.kt` — documented measured/target
  rotations and the shared clamp; removed an unused import.
- `src/main/kotlin/com/areslib/frc/marvin/MarvinFeederController.kt` — documented firing interlocks
  and transfer ownership, replaced a duplicated shoot-speed literal with the shared constant, and
  flattened equivalent feeder/floor dispatch branches.
- `src/main/kotlin/com/areslib/frc/marvin/MarvinFlywheelController.kt` — documented RPM targets,
  stop/re-command semantics, and fail-closed readiness; removed an unused import.
- `src/main/kotlin/com/areslib/frc/marvin/MarvinIntakeSubsystem.kt` — distinguished logical deploy
  state from measured pivot degrees and documented roller RPS.
- `src/main/kotlin/com/areslib/frc/marvin/MarvinReducer.kt` — replaced an inaccurate zero-allocation
  claim with the reducer's actual deadband/freshness contract and documented pure composition.
- `src/main/kotlin/com/areslib/frc/marvin/MarvinShooterFacade.kt` — documented measured field-frame
  SOTM inputs, caller-owned scratch output, cowl rotations, rearward aim, and firing gates; named
  acceleration-lookahead and aim-gain literals and removed unused imports.
- `src/main/kotlin/com/areslib/frc/marvin/MarvinState.kt` — documented all mechanism units, validity,
  last-trusted detector state, climber mode selection, flywheel readiness, and custom-state fallback.
- `src/main/kotlin/com/areslib/frc/marvin/MarvinSuperstructure.kt` — documented the read-once/write-
  outputs loop contract, freshness propagation, and brownout geometry; flattened the equivalent
  slamtake guard and named feeder/floor voltage-per-RPS constants.

### Teleop and autonomous orchestration

- `src/main/kotlin/com/areslib/frc/robot/FRCAutoOrchestrator.kt` — reduced orchestration to native
  `.aresauto` resolution, field-footprint preflight, alliance transformation, shared compilation,
  deterministic task execution, pose seeding, telemetry, and fail-safe lifecycle cleanup.
- `src/main/kotlin/com/areslib/frc/robot/FRCTeleOpDriveController.kt` — documented meters/radians,
  red-alliance translation rotation, cached input ownership, command priority, Redux synchronization,
  and allocation avoidance; replaced a false slamtake comment and simplified enum mapping.

### Simulation implementation

- `src/main/kotlin/com/areslib/frc/sim/Dyn4jPhysicsWorld.kt` — documented world/body ownership,
  meters/radians, retained robot-body behavior, and step/reset contracts; removed unused imports.
- `src/main/kotlin/com/areslib/frc/sim/Dyn4jSimTelemetryPublisher.kt` — documented pose-array layout,
  quaternion ordering, reusable buffers, high-water growth, and stale-entry clearing.
- `src/main/kotlin/com/areslib/frc/sim/Dyn4jSwerveModuleSim.kt` — documented robot-to-world velocity
  rotation, CCW angular units, force ownership, and reuse of the force vector.
- `src/main/kotlin/com/areslib/frc/sim/field/FrcFieldBuilder.kt` — documented canonical field extents
  and explicitly labeled interior collision bodies as approximations rather than official drawings.
- `src/main/kotlin/com/areslib/frc/sim/io/SimulatedClimberIO.kt` — documented mechanism rotations and
  geometry-preserving effort scaling.
- `src/main/kotlin/com/areslib/frc/sim/io/SimulatedCowlIO.kt` — documented public rotations, internal
  degree representation, the 32-degree mapping, and effort scaling.
- `src/main/kotlin/com/areslib/frc/sim/io/SimulatedFeederIO.kt` — documented optional detector
  validity and fail-closed interpretation.
- `src/main/kotlin/com/areslib/frc/sim/io/SimulatedFloorIO.kt` — documented voltage and mechanism RPS.
- `src/main/kotlin/com/areslib/frc/sim/io/SimulatedFlywheelIO.kt` — documented RPM, voltage clamping,
  and why in-process velocity is always valid.
- `src/main/kotlin/com/areslib/frc/sim/io/SimulatedIntakeIO.kt` — documented pivot degrees, roller
  voltage, and geometry-preserving effort scaling.

## Changed test files (8)

- `src/test/kotlin/com/areslib/frc/ARESRobotTest.kt` — removed generated placeholders and an unmapped
  copilot-A step that claimed to toggle the intake but asserted no behavior.
- `src/test/kotlin/com/areslib/frc/AresFrcRemediationTest.kt` — corrected rearward-heading reasoning,
  removed an unused value, and clarified the rotations API assertion.
- `src/test/kotlin/com/areslib/frc/Dyn4jSimulationTest.kt` — corrected detector-edge, cowl-unit, and
  internal-versus-public representation comments; removed generated placeholders.
- `src/test/kotlin/com/areslib/frc/reducer/MarvinReducerTest.kt` — removed false claims that climber
  commands were intake-clamped or that a CBF lived in the shooter facade; made phase and unit
  comments match the actual actions.
- `src/test/kotlin/com/areslib/frc/robot/FrcNativeAutoContractTest.kt` — checks manifest/runtime
  catalog parity, every deploy asset on both alliances, field rejection, pose mirroring, readiness
  timeout, and execution through the production native runner.
- `src/test/kotlin/com/areslib/frc/robot/FRCTeleOpDriveControllerTest.kt` — renamed claims to match
  chassis-command coverage and clarified where field-frame transformation occurs.
- `src/test/kotlin/com/areslib/frc/sim/field/FrcFieldBuilderTest.kt` — removed generated placeholders.
- `src/test/kotlin/com/areslib/frc/sim/io/SimulatedIOTest.kt` — corrected the cowl feedback assertion
  to compare internal degrees with public mechanism rotations and removed placeholders.

## Reviewed without changes (10)

- `src/test/kotlin/com/areslib/frc/ARESRobotTimedBehaviorRegressionTest.kt`
- `src/test/kotlin/com/areslib/frc/marvin/MarvinConfigFieldGeometryTest.kt`
- `src/test/kotlin/com/areslib/frc/marvin/MarvinControlAndFreshnessRegressionTest.kt`
- `src/test/kotlin/com/areslib/frc/marvin/MarvinControllerReduxConsistencyTest.kt`
- `src/test/kotlin/com/areslib/frc/marvin/MarvinMeasuredSotmRegressionTest.kt`
- `src/test/kotlin/com/areslib/frc/marvin/MarvinSuperstructureSafetyTest.kt`
- `src/test/kotlin/com/areslib/frc/robot/FRCAutoAllianceMirroringContractTest.kt`
- `src/test/kotlin/com/areslib/frc/robot/FRCAutoWaitStateRegressionTest.kt`
- `src/test/kotlin/com/areslib/frc/sim/field/FrcFieldGeometryContractTest.kt`
- `src/test/kotlin/com/areslib/frc/sim/io/SimulatedSafetyContractTest.kt`

## Suspected behavior defects intentionally not changed

These findings need design/behavior decisions and were kept outside this readability-only pass.

1. **High — no reciprocal climber/intake collision interlock.**
   `src/main/kotlin/com/areslib/frc/marvin/MarvinSuperstructure.kt:75` writes the requested intake
   pivot directly and line 86 independently selects climber output mode. The reducer test previously
   claimed a CBF had moved to `MarvinShooterFacade`, but that facade contains no climber/intake
   constraint. A simultaneous stow and extension request can therefore reach both IO boundaries.
   Minimal fix: enforce the calibrated collision envelope once at the output boundary, using cached
   measured geometry and explicit safe fallback, then add reciprocal direction tests.

2. **High — the default Dyn4j interior layout is not a Crescendo field.**
   `src/main/kotlin/com/areslib/frc/sim/field/FrcFieldBuilder.kt:28` onward installs hubs, four towers,
   trench barriers, and climb bases; `Dyn4jPhysicsWorld.kt:92` onward spawns pieces around hubs and
   trenches. Those are not official 2024 Crescendo structures or note locations, so collision and
   autonomous simulation can validate against the wrong environment. Minimal fix: build the stage,
   sources, subwoofers, and initial notes from official Crescendo dimensions, preferably through a
   data-driven field configuration.

3. **Medium — static and moving shot geometry use different origins.**
   `src/main/kotlin/com/areslib/frc/marvin/MarvinShooterFacade.kt:135` computes static lookup distance
   from robot center and manually reconstructs rearward heading, while SOTM delegates shooter
   offsets and delay compensation to `ShotSetup`. Switching modes at the same stationary pose can
   therefore change RPM/cowl/heading. Minimal fix: use `ShotSetup.calculate` with zero chassis speeds
   and a retained `ShotResult` for the static path as well.

4. **Medium — the simulation constructor seed does not seed every random stream.**
   `src/main/kotlin/com/areslib/frc/Dyn4jSimulation.kt:93` always creates `Random(42L)` even though the
   constructor accepts `seed` and passes it to the physics world. Different requested seeds produce
   identical scoring ejection randomness. Minimal fix: construct this random source with `seed` and
   add same-seed/different-seed determinism tests.

5. **Medium — a 50 Hz telemetry publisher allocates on every robot loop.**
   `src/main/kotlin/com/areslib/frc/ARESRobot.kt:218` creates a new `doubleArrayOf` for packed
   superstructure telemetry on every store update, contrary to the workspace zero-GC loop contract.
   Minimal fix: retain a fixed `DoubleArray`, overwrite its slots, and publish that buffer if the
   telemetry implementation copies synchronously (otherwise use an ownership-safe pool).

6. **Low — rebuilding a simulation field does not clear airborne pieces.**
   `src/main/kotlin/com/areslib/frc/sim/Dyn4jPhysicsWorld.kt:49` removes Dyn4j bodies and clears
   grounded `balls`, but leaves `flyingBalls` intact. A runtime field rebuild can carry projectiles
   from the previous field into the new one. Minimal fix: clear `flyingBalls` in `buildWorld` and add
   a rebuild-state regression.
