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

/** Generated from the project's checked-in ARES documents. Do not edit by hand. */
object GeneratedAresProject {
    const val GENERATOR_VERSION: Int = 5
    const val CATALOG_SHA256: String = "19ed9bc352df84bfeb33770fb1cb7b3507de57d4e80f38a4ed4427affab97246"
    const val CONTENT_SHA256: String = "984fe9ed5848d0ebf9002a25ac93a2e3bb7f00696c2692c462bbb999f6becc96"
    const val SOURCE_SHA256: String = "6f425a5c4f9d8a40199a89593ac1cf5f4c50c91b85ed3f653c4465d0b4628f8f"

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
                    "shooter.feedWhenReady" -> setOf("floor", "shooter.cowl", "shooter.feeder", "shooter.flywheel")
                    "shooter.prepare" -> setOf("shooter.cowl", "shooter.flywheel")
                    "shooter.stop" -> setOf("floor", "shooter.feeder", "shooter.flywheel")
                    else -> emptySet()
                }
            },
        )

}
