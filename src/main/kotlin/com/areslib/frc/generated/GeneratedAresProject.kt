@file:Suppress("MagicNumber", "LongMethod")

package com.areslib.frc.generated

import com.areslib.codegen.CapabilityArgumentReader
import com.areslib.routine.AutonomousCatalogEntry
import com.areslib.routine.RoutineDocument
import com.areslib.routine.RoutineDriveMarker
import com.areslib.routine.RoutineDriveStep
import com.areslib.routine.RoutinePose
import com.areslib.routine.RoutineRuntimeBindings
import com.areslib.routine.RoutineStep
import com.areslib.routine.RoutineStepKind
import com.areslib.routine.RoutineManager
import com.areslib.input.ControllerBindingRuntime
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
import com.areslib.input.DigitalBinding
import com.areslib.input.DigitalBindingListener
import com.areslib.input.DigitalBindingTiming
import com.areslib.input.RawButtonSource
import com.areslib.input.SuppressibleButtonSource
import com.areslib.input.SuppressingButtonChordSource
import com.areslib.input.ThresholdDirection
import com.areslib.sequencer.Task
import com.areslib.state.RobotState

/** Stable robot boundary for capabilities referenced by generated project documents. */
interface GeneratedAresProjectCapabilities {
    /** Creates a hand-authored or season action by its catalog key, or null when unavailable. */
    fun createActionTask(actionKey: String, arguments: Map<String, String>): Task? = null

    /**
     * Receives the combined teleop drivetrain command once per frame. Values are normalized
     * (-1..1) after each axis binding's transform, field-centric with CCW-positive rotation;
     * the implementation scales them by drivetrain limits and applies alliance mirroring.
     * [active] is false when the scheme has no drive bindings, so sinks without generated
     * drivetrain control stay inert.
     */
    fun onDriveCommand(vx: Double, vy: Double, omega: Double, active: Boolean) = Unit

    /** Creates a hand-authored condition predicate by its catalog key, or null when unavailable. */
    fun createCondition(conditionKey: String, arguments: Map<String, String>): ((RobotState) -> Boolean)? = null

    /** Platform trajectory adapter; returning null rejects a drive step safely. */
    fun createDriveTask(step: RoutineDriveStep): Task? = null
}

/** Robot scheduler boundary used by generated direct-action controller bindings. */
fun interface GeneratedAresProjectControlTaskSink {
    fun submit(bindingId: String, task: Task)
}

/** Generated from the project's checked-in ARES documents. Do not edit by hand. */
object GeneratedAresProject {
    const val GENERATOR_VERSION: Int = 8
    const val CATALOG_SHA256: String = "19ed9bc352df84bfeb33770fb1cb7b3507de57d4e80f38a4ed4427affab97246"
    const val CONTENT_SHA256: String = "54584a6357dcaf6ed1390a9696ebd0d4d4543f33678ef5d7ade7464a130a6e97"
    const val SOURCE_SHA256: String = "fb8932c603ecd3169c3646749f5140d5dd42ab159851fb7c1c48410ab0bf1552"

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
            schemaVersion = 2,
            documentId = "do-nothing",
            revision = 1,
            parentContentHash = null,
            name = "Do Nothing",
            description = "Match-safe routine that intentionally leaves the robot stationary.",
            steps = listOf(
                RoutineStep(
                    kind = RoutineStepKind.WAIT,
                    stepId = "step-hold-position",
                    durationSeconds = 0.0,
                ),
            ),
        ),
    )

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
                        registry.createActionTask(key, arguments)
                    }
                    "intake.stop" -> {
                        CapabilityArgumentReader(
                            capabilityKey = "intake.stop",
                            arguments = arguments,
                            allowedKeys = emptySet(),
                        )
                        registry.createActionTask(key, arguments)
                    }
                    "intake.stow" -> {
                        CapabilityArgumentReader(
                            capabilityKey = "intake.stow",
                            arguments = arguments,
                            allowedKeys = emptySet(),
                        )
                        registry.createActionTask(key, arguments)
                    }
                    "shooter.feedWhenReady" -> {
                        CapabilityArgumentReader(
                            capabilityKey = "shooter.feedWhenReady",
                            arguments = arguments,
                            allowedKeys = emptySet(),
                        )
                        registry.createActionTask(key, arguments)
                    }
                    "shooter.prepare" -> {
                        CapabilityArgumentReader(
                            capabilityKey = "shooter.prepare",
                            arguments = arguments,
                            allowedKeys = emptySet(),
                        )
                        registry.createActionTask(key, arguments)
                    }
                    "shooter.stop" -> {
                        CapabilityArgumentReader(
                            capabilityKey = "shooter.stop",
                            arguments = arguments,
                            allowedKeys = emptySet(),
                        )
                        registry.createActionTask(key, arguments)
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
                        registry.createCondition(key, arguments)
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
                    "shooter.feedWhenReady" -> setOf("floor", "shooter.feeder")
                    "shooter.prepare" -> setOf("shooter.cowl", "shooter.flywheel")
                    "shooter.stop" -> setOf("floor", "shooter.feeder", "shooter.flywheel")
                    else -> emptySet()
                }
            },
        )

    val knownControlSchemeIds: Set<String> = setOf("driver")
    val DEFAULT_CONTROL_SCHEME_ID: String? = "driver"

    /** True when the active scheme binds at least one drivetrain axis. */
    val HAS_GENERATED_DRIVE_BINDINGS: Boolean = true
    private val driveAxisValues = DoubleArray(3)

    /**
     * Publishes the latest drive-axis listener values as one combined command. Disconnects emit
     * zeros and the analog rearm policy holds that neutral until every axis passes through its
     * deadband, so a deflected stick cannot lurch the robot across a controller reconnect.
     */
    fun emitDriveCommand(registry: GeneratedAresProjectCapabilities) {
        registry.onDriveCommand(driveAxisValues[0], driveAxisValues[1], driveAxisValues[2], HAS_GENERATED_DRIVE_BINDINGS)
    }

    /**
     * Builds one allocation-free update runtime per zero-based Driver Station port. Suppressing chords are
     * ordered before constituent buttons and raise their effective press debounce to the chord
     * window, preventing a near-simultaneous chord from leaking a single-button action.
     */
    @Suppress("UNUSED_PARAMETER")
    fun createControllerRuntimes(
        schemeId: String?,
        registry: GeneratedAresProjectCapabilities,
        routineManager: RoutineManager,
        taskSink: GeneratedAresProjectControlTaskSink,
    ): Map<Int, ControllerBindingRuntime> {
        val activeSchemeId = requireNotNull(schemeId) { "A generated control scheme is required" }
        return when (activeSchemeId) {
        "driver" -> run {
            val buttonSuppression_driver_b4def821 = ButtonSuppressionState(buttonCapacity = 128)
            linkedMapOf(
                0 to ControllerBindingRuntime(
                    digitalBindings = emptyList(),
                    analogBindings = listOf(
                        AnalogBinding(
                            axisIndex = 1,
                            transform = AxisTransform(
                                inputMin = -1.0,
                                inputCenter = 0.0,
                                inputMax = 1.0,
                                deadband = 0.1,
                                exponent = 1.0,
                                inverted = true,
                                outputMin = -1.0,
                                outputMax = 1.0,
                            ),
                            listener = object : AnalogBindingListener {
                                override fun onValue(value: Double) {
                                    driveAxisValues[0] = value
                                }
                            },
                            zones = emptyList(),
                            emissionPolicy = AnalogEmissionPolicy.EVERY_UPDATE,
                            changeEpsilon = 1.0E-6,
                            riseRatePerSecond = Double.POSITIVE_INFINITY,
                            fallRatePerSecond = Double.POSITIVE_INFINITY,
                            rearmNeutralThreshold = 0.05,
                        ),
                        AnalogBinding(
                            axisIndex = 4,
                            transform = AxisTransform(
                                inputMin = -1.0,
                                inputCenter = 0.0,
                                inputMax = 1.0,
                                deadband = 0.1,
                                exponent = 1.0,
                                inverted = true,
                                outputMin = -1.0,
                                outputMax = 1.0,
                            ),
                            listener = object : AnalogBindingListener {
                                override fun onValue(value: Double) {
                                    driveAxisValues[2] = value
                                }
                            },
                            zones = emptyList(),
                            emissionPolicy = AnalogEmissionPolicy.EVERY_UPDATE,
                            changeEpsilon = 1.0E-6,
                            riseRatePerSecond = Double.POSITIVE_INFINITY,
                            fallRatePerSecond = Double.POSITIVE_INFINITY,
                            rearmNeutralThreshold = 0.05,
                        ),
                        AnalogBinding(
                            axisIndex = 0,
                            transform = AxisTransform(
                                inputMin = -1.0,
                                inputCenter = 0.0,
                                inputMax = 1.0,
                                deadband = 0.1,
                                exponent = 1.0,
                                inverted = true,
                                outputMin = -1.0,
                                outputMax = 1.0,
                            ),
                            listener = object : AnalogBindingListener {
                                override fun onValue(value: Double) {
                                    driveAxisValues[1] = value
                                }
                            },
                            zones = emptyList(),
                            emissionPolicy = AnalogEmissionPolicy.EVERY_UPDATE,
                            changeEpsilon = 1.0E-6,
                            riseRatePerSecond = Double.POSITIVE_INFINITY,
                            fallRatePerSecond = Double.POSITIVE_INFINITY,
                            rearmNeutralThreshold = 0.05,
                        ),
                    ),
                ),
            )
        }
            else -> throw IllegalArgumentException("Unknown control scheme '$activeSchemeId'")
        }
    }
}
