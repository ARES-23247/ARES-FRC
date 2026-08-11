@file:Suppress("MagicNumber", "LongMethod")

package com.areslib.frc.generated

import com.areslib.codegen.CapabilityArgumentReader
import com.areslib.routine.AutonomousCatalogEntry
import com.areslib.routine.AutonomousRoutineEntryPoint
import com.areslib.routine.RoutineDocument
import com.areslib.routine.RoutineDriveMarker
import com.areslib.routine.RoutineDriveStep
import com.areslib.routine.RoutinePose
import com.areslib.routine.RoutineRuntimeBindings
import com.areslib.routine.RoutineStep
import com.areslib.routine.RoutineStepKind
import com.areslib.routine.RoutineManager
import com.areslib.routine.RoutineStartPolicy
import com.areslib.input.AnalogBinding
import com.areslib.input.AnalogBindingListener
import com.areslib.input.AnalogEmissionPolicy
import com.areslib.input.AnalogZone
import com.areslib.input.AnalogZoneListener
import com.areslib.input.AxisThresholdSource
import com.areslib.input.AxisTransform
import com.areslib.input.BindingReleaseReason
import com.areslib.input.ButtonSuppressionState
import com.areslib.input.ChordSource
import com.areslib.input.ControllerBindingRuntime
import com.areslib.input.DigitalBinding
import com.areslib.input.DigitalBindingListener
import com.areslib.input.DigitalBindingTiming
import com.areslib.input.RawButtonSource
import com.areslib.input.SuppressibleButtonSource
import com.areslib.input.SuppressingButtonChordSource
import com.areslib.input.ThresholdDirection
import com.areslib.sequencer.Task
import com.areslib.state.RobotState

/** Typed robot implementations for every capability in the generated catalog. */
interface GeneratedAresProjectCapabilities {
    /** Implements action key intake.collect. */
    fun actionIntakeCollect(): Task

    /** Implements action key intake.stop. */
    fun actionIntakeStop(): Task

    /** Implements action key intake.stow. */
    fun actionIntakeStow(): Task

    /** Implements action key shooter.feedWhenReady. */
    fun actionShooterFeedWhenReady(): Task

    /** Implements action key shooter.prepare. */
    fun actionShooterPrepare(): Task

    /** Implements action key shooter.stop. */
    fun actionShooterStop(): Task

    /** Implements condition key shooter.ready. */
    fun conditionShooterReady(): (RobotState) -> Boolean

    /** Platform trajectory adapter; returning null rejects a drive step safely. */
    fun createDriveTask(step: RoutineDriveStep): Task? = null
}

/** Robot scheduler boundary used by generated direct-action controller bindings. */
fun interface GeneratedAresProjectControlTaskSink {
    fun submit(bindingId: String, task: Task)
}

/** Generated from the project's checked-in ARES documents. Do not edit by hand. */
object GeneratedAresProject {
    const val GENERATOR_VERSION: Int = 2
    const val CATALOG_SHA256: String = "c0355427c078db051a1c3b2750f6a77328b38cc29414d7acc71cbfb019cbad0d"
    const val CONTENT_SHA256: String = "1cefbce71b115c4bd113902ee093b28567472fc5a8a85423e36f3dd1075e4803"
    const val SOURCE_SHA256: String = "716e7d6a74b8655a5f63d2e2434d75f249924a3c82d7d569f6c370a2c4a3e710"

    const val PROJECT_ID: String = "team23247-marvin-xix"
    const val PROJECT_LEAGUE: String = "FRC"
    const val COORDINATE_CONVENTION: String = "BLUE_CORNER_ORIGIN_CCW"
    const val ROBOT_LENGTH_METERS: Double = 0.8
    const val ROBOT_WIDTH_METERS: Double = 0.8
    const val FIELD_LENGTH_METERS: Double = 16.54175
    const val FIELD_WIDTH_METERS: Double = 8.21055

    val knownActionKeys: Set<String> = setOf("intake.collect", "intake.stop", "intake.stow", "shooter.feedWhenReady", "shooter.prepare", "shooter.stop")
    val knownConditionKeys: Set<String> = setOf("shooter.ready")

    val routines: Map<String, RoutineDocument> = linkedMapOf(
        "do-nothing" to RoutineDocument(
            schemaVersion = 1,
            documentId = "do-nothing",
            revision = 1,
            parentContentHash = null,
            name = "Do Nothing",
            description = "Match-safe routine that intentionally leaves the robot stationary.",
            steps = listOf(
                RoutineStep(
                    kind = RoutineStepKind.WAIT,
                    durationSeconds = 0.0,
                ),
            ),
        ),
        "sim-drive-and-shoot" to RoutineDocument(
            schemaVersion = 1,
            documentId = "sim-drive-and-shoot",
            revision = 1,
            parentContentHash = null,
            name = "Simulation Drive and Shoot",
            description = "Exercises field motion, markers, readiness gating, and safe cleanup.",
            steps = listOf(
                RoutineStep(
                    kind = RoutineStepKind.DRIVE_TO,
                    drive = RoutineDriveStep(
                        target = RoutinePose(
                            xMeters = 3.6,
                            yMeters = 2.65,
                            headingRadians = 0.0,
                        ),
                        motionPresetKey = "safe",
                        preferredEngineKey = null,
                        markers = listOf(
                            RoutineDriveMarker(progress = 0.1, actionKey = "shooter.prepare"),
                            RoutineDriveMarker(progress = 0.45, actionKey = "intake.collect"),
                        ),
                        arrivalActionKeys = listOf("shooter.feedWhenReady"),
                    ),
                ),
                RoutineStep(
                    kind = RoutineStepKind.WAIT,
                    durationSeconds = 0.5,
                ),
                RoutineStep(
                    kind = RoutineStepKind.ACTION,
                    actionKey = "shooter.stop",
                ),
                RoutineStep(
                    kind = RoutineStepKind.ACTION,
                    actionKey = "intake.stow",
                ),
            ),
        ),
    )

    val autonomousEntryPoints: Map<String, AutonomousRoutineEntryPoint> = linkedMapOf()

    val autonomousEntries: List<AutonomousCatalogEntry> = listOf(
        AutonomousCatalogEntry(
            entryId = "do-nothing",
            displayName = "Do Nothing",
            description = "Safe fallback; hold position and run no mechanisms.",
            routineId = "do-nothing",
            startingPose = RoutinePose(
                xMeters = 0.5,
                yMeters = 0.5,
                headingRadians = 0.0,
            ),
            authoredAlliance = com.areslib.routine.RoutineAlliance.BLUE,
            mirrorForOppositeAlliance = true,
            sortOrder = 0,
            enabled = true,
        ),
        AutonomousCatalogEntry(
            entryId = "sim-drive-and-shoot",
            displayName = "Simulation Drive and Shoot",
            description = "Drive, prepare, collect, feed, and stop.",
            routineId = "sim-drive-and-shoot",
            startingPose = RoutinePose(
                xMeters = 2.0,
                yMeters = 2.0,
                headingRadians = 0.0,
            ),
            authoredAlliance = com.areslib.routine.RoutineAlliance.BLUE,
            mirrorForOppositeAlliance = true,
            sortOrder = 1,
            enabled = true,
        ),
    )
    val DEFAULT_AUTONOMOUS_ENTRY_ID: String? = "do-nothing"

    fun runtimeBindings(registry: GeneratedAresProjectCapabilities): RoutineRuntimeBindings =
        RoutineRuntimeBindings(
            createActionTask = { key, arguments ->
                when (key) {
                    "intake.collect" -> {
                        CapabilityArgumentReader(
                            capabilityKey = "intake.collect",
                            arguments = arguments,
                            allowedKeys = emptySet(),
                        )
                        registry.actionIntakeCollect()
                    }
                    "intake.stop" -> {
                        CapabilityArgumentReader(
                            capabilityKey = "intake.stop",
                            arguments = arguments,
                            allowedKeys = emptySet(),
                        )
                        registry.actionIntakeStop()
                    }
                    "intake.stow" -> {
                        CapabilityArgumentReader(
                            capabilityKey = "intake.stow",
                            arguments = arguments,
                            allowedKeys = emptySet(),
                        )
                        registry.actionIntakeStow()
                    }
                    "shooter.feedWhenReady" -> {
                        CapabilityArgumentReader(
                            capabilityKey = "shooter.feedWhenReady",
                            arguments = arguments,
                            allowedKeys = emptySet(),
                        )
                        registry.actionShooterFeedWhenReady()
                    }
                    "shooter.prepare" -> {
                        CapabilityArgumentReader(
                            capabilityKey = "shooter.prepare",
                            arguments = arguments,
                            allowedKeys = emptySet(),
                        )
                        registry.actionShooterPrepare()
                    }
                    "shooter.stop" -> {
                        CapabilityArgumentReader(
                            capabilityKey = "shooter.stop",
                            arguments = arguments,
                            allowedKeys = emptySet(),
                        )
                        registry.actionShooterStop()
                    }
                    else -> null
                }
            },
            createCondition = { key, arguments ->
                when (key) {
                    "shooter.ready" -> {
                        CapabilityArgumentReader(
                            capabilityKey = "shooter.ready",
                            arguments = arguments,
                            allowedKeys = emptySet(),
                        )
                        registry.conditionShooterReady()
                    }
                    else -> null
                }
            },
            createDriveTask = registry::createDriveTask,
            isActionKnown = knownActionKeys::contains,
            isConditionKnown = knownConditionKeys::contains,
            resourcesForAction = { key ->
                when (key) {
                    "intake.collect" -> setOf("floor", "intake")
                    "intake.stop" -> setOf("floor", "intake")
                    "intake.stow" -> setOf("floor", "intake")
                    "shooter.feedWhenReady" -> setOf("floor", "shooter.feeder", "shooter.flywheel")
                    "shooter.prepare" -> setOf("shooter.flywheel")
                    "shooter.stop" -> setOf("floor", "shooter.feeder", "shooter.flywheel")
                    else -> emptySet()
                }
            },
        )

    val knownControlSchemeIds: Set<String> = emptySet()

    /**
     * Builds one allocation-free update runtime per controller slot. Suppressing chords are
     * ordered before constituent buttons and raise their effective press debounce to the chord
     * window, preventing a near-simultaneous chord from leaking a single-button action.
     */
    @Suppress("UNUSED_PARAMETER")
    fun createControllerRuntimes(
        schemeId: String,
        registry: GeneratedAresProjectCapabilities,
        routineManager: RoutineManager,
        taskSink: GeneratedAresProjectControlTaskSink,
    ): Map<String, ControllerBindingRuntime> {
        throw IllegalArgumentException("Unknown control scheme '$schemeId'")
    }
}
