package com.areslib.frc.robot

import com.areslib.action.RobotAction
import com.areslib.frc.Dyn4jSimulation
import com.areslib.frc.FrcSwerveRobot
import com.areslib.frc.generated.GeneratedAresProject
import com.areslib.frc.generatedruntime.FrcControllerBindingHost
import com.areslib.frc.generatedruntime.FrcControllerBindingSlot
import com.areslib.frc.generatedruntime.FrcGeneratedRoutineCapabilities
import com.areslib.frc.generatedruntime.FrcGeneratedControlTaskScheduler
import com.areslib.frc.generatedruntime.FrcGeneratedControllerPorts
import com.areslib.frc.generatedruntime.requireFrcRoutinePoseInsideField
import com.areslib.frc.marvin.SetClimberVoltage
import com.areslib.frc.marvin.StopSlamtake
import com.areslib.math.geometry.Pose2d
import com.areslib.routine.AutonomousCatalogEntry
import com.areslib.routine.RoutineManager
import com.areslib.routine.RoutineRequestResult
import com.areslib.routine.RoutineStartPolicy
import com.areslib.routine.RoutineStep
import com.areslib.state.RoutineExecutionStatus
import com.areslib.util.RobotClock

/** Result of resolving a dashboard request against the checked-in generated autonomous catalog. */
data class FrcAutonomousSelection(
    val entry: AutonomousCatalogEntry,
    val requestedId: String,
    val usedFallback: Boolean
)

/** Pure selector that filters disabled entries and guarantees a deterministic safe fallback. */
class FrcAutonomousSelector(
    entries: List<AutonomousCatalogEntry>,
    defaultEntryId: String?
) {
    private val enabledEntries = entries.filter(AutonomousCatalogEntry::enabled)
        .sortedWith(compareBy<AutonomousCatalogEntry> { it.sortOrder }.thenBy { it.entryId })
    private val entriesById = enabledEntries.associateBy(AutonomousCatalogEntry::entryId)
    private val fallback = defaultEntryId?.let(entriesById::get)
        ?: entriesById[SAFE_FALLBACK_ENTRY_ID]
        ?: enabledEntries.firstOrNull()

    val availableEntryIds: List<String> = enabledEntries.map(AutonomousCatalogEntry::entryId)

    fun resolve(requestedId: String): FrcAutonomousSelection {
        val normalized = requestedId.trim()
        val requested = entriesById[normalized]
        val selected = requested ?: checkNotNull(fallback) {
            "Generated autonomous catalog has no enabled fail-safe entry"
        }
        return FrcAutonomousSelection(
            entry = selected,
            requestedId = normalized,
            usedFallback = requested == null
        )
    }

    private companion object {
        const val SAFE_FALLBACK_ENTRY_ID = "do-nothing"
    }
}

/**
 * Executes one generated ARES routine during the FRC autonomous period.
 *
 * Selection is sampled and locked exactly once in [autonomousInit]. Every routine and autonomous
 * entry is compiled into the robot program; deploy-time `.aresauto`, PathPlanner, and Choreo files
 * are import compatibility only. Missing selections fall back to the generated do-nothing entry,
 * while invalid catalogs, field poses, task compilation, and runtime failures fail closed.
 */
class FRCAutoOrchestrator @JvmOverloads constructor(
    private val robot: FrcSwerveRobot,
    private val sim: Dyn4jSimulation? = null,
    private val selectionProvider: () -> String = ::dashboardSelection
) {
    private val selector = FrcAutonomousSelector(
        GeneratedAresProject.autonomousEntries,
        GeneratedAresProject.DEFAULT_AUTONOMOUS_ENTRY_ID
    )
    private val capabilities = FrcGeneratedRoutineCapabilities(robot)
    private val routineManager = RoutineManager(
        bindings = GeneratedAresProject.runtimeBindings(capabilities),
        stateProvider = { robot.store.state },
        dispatch = robot.store::dispatch
    ).also { manager -> manager.replaceDocuments(GeneratedAresProject.routines.values) }

    private var activeExecutionId: Long? = null
    private var autoFaulted = false
    private var finished = true
    private var selectedAutoId = GeneratedAresProject.DEFAULT_AUTONOMOUS_ENTRY_ID ?: "do-nothing"
    private var status = "Idle"

    /** Publishes generated choices and initializes the dashboard selection without robot IO. */
    fun publishCatalog() {
        runCatching {
            val table = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
                .getTable(SMART_DASHBOARD_TABLE)
            table.getEntry(SELECTED_AUTO_ENTRY).setDefaultString(selectedAutoId)
            table.getEntry(AVAILABLE_AUTOS_ENTRY).setStringArray(selector.availableEntryIds.toTypedArray())
        }
        robot.telemetry.putString(
            "ARES/Auto/AvailableDocuments",
            selector.availableEntryIds.joinToString(",")
        )
        robot.telemetry.putString("ARES/Auto/Source", "generated:${GeneratedAresProject.CONTENT_SHA256}")
    }

    /**
     * Creates the complete generated teleop graph around the same routine manager used by auto.
     * Driver and operator slots map deterministically to DS ports 0 and 1; any other slot fails
     * installation so robot initialization can retain the legacy controls safely.
     */
    fun createControllerBindingHost(schemeId: String): FrcControllerBindingHost {
        val taskScheduler = FrcGeneratedControlTaskScheduler(
            stateProvider = { robot.store.state },
            dispatch = robot.store::dispatch
        )
        val runtimes = GeneratedAresProject.createControllerRuntimes(
            schemeId = schemeId,
            registry = capabilities,
            routineManager = routineManager,
            taskSink = taskScheduler
        )
        require(runtimes.isNotEmpty()) { "Generated scheme '$schemeId' has no controller slots" }
        val slots = runtimes.entries.sortedBy { it.key }.map { (slotId, runtime) ->
            FrcControllerBindingSlot(
                slotId = slotId,
                port = FrcGeneratedControllerPorts.resolve(slotId),
                runtime = runtime
            )
        }
        return FrcControllerBindingHost(
            slots = slots,
            afterBindingsUpdate = {
                taskScheduler.update()
                routineManager.update()
            },
            afterBindingsCancel = {
                taskScheduler.cancel()
                routineManager.cancelAll("Generated controls cancelled")
            }
        )
    }

    /** True while a controller-started routine owns a generated drive step. */
    fun generatedRoutineOwnsDrive(): Boolean {
        val executions = robot.store.state.routineState.executions
        if (executions.isEmpty()) return false
        return executions.values.any { it.activeStepKind == "DRIVE_TO" }
    }

    /** Locks, validates, alliance-transforms, seeds, and requests the selected generated routine. */
    fun autonomousInit() {
        cancelActive("Autonomous reinitialized")
        autoFaulted = false
        finished = false

        try {
            val selection = selector.resolve(selectionProvider())
            val entry = selection.entry
            selectedAutoId = entry.entryId
            capabilities.configure(entry, robot.store.state.drive.alliance)
            validateFieldBounds(entry)

            if (selection.usedFallback) {
                robot.telemetry.putString(
                    "ARES/Auto/Warning/1",
                    "Requested '${selection.requestedId}' is unavailable; using '${entry.entryId}'"
                )
            }

            seedPose(capabilities.transform(entry.startingPose))
            when (val result = routineManager.request(entry.routineId, RoutineStartPolicy.RESTART_EXISTING)) {
                is RoutineRequestResult.Accepted -> activeExecutionId = result.executionId
                is RoutineRequestResult.AlreadyRunning -> activeExecutionId = result.executionId
                is RoutineRequestResult.Rejected -> error(
                    result.issues.joinToString(separator = "; ") { it.message }
                )
            }
            setStatus(if (selection.usedFallback) "Running safe fallback" else "Running")
        } catch (error: Exception) {
            abort("Preflight failed: ${error.message ?: error::class.java.simpleName}")
        }
    }

    /** Advances the single shared routine manager and observes its Redux terminal lifecycle. */
    fun autonomousPeriodic() {
        if (finished || autoFaulted) return
        val executionId = activeExecutionId ?: run {
            abort("Autonomous routine was not armed")
            return
        }

        try {
            routineManager.update()
            val active = robot.store.state.routineState.executions[executionId]
            if (active != null) {
                robot.telemetry.putString(
                    "ARES/Auto/ActiveTask",
                    active.activeStepPath ?: active.routineId
                )
                return
            }

            val terminal = robot.store.state.routineState.lastTerminalExecution
            if (terminal?.executionId != executionId) {
                abort("Routine lifecycle ended without a matching terminal result")
                return
            }
            when (terminal.status) {
                RoutineExecutionStatus.COMPLETED -> complete()
                RoutineExecutionStatus.FAILED -> abort(terminal.message ?: "Routine task failed")
                RoutineExecutionStatus.CANCELLED -> abort(terminal.message ?: "Routine was cancelled")
                RoutineExecutionStatus.REQUESTED,
                RoutineExecutionStatus.RUNNING -> abort("Routine left the active set before completion")
            }
        } catch (error: Exception) {
            abort("Runtime failed: ${error.message ?: error::class.java.simpleName}")
        }
    }

    /** Cancels every active/queued routine and drives all outputs to their fail-safe state. */
    fun stop() {
        cancelActive("Robot disabled or autonomous exited")
        finished = true
        failSafeStop()
        setStatus("Stopped")
    }

    private fun complete() {
        finished = true
        activeExecutionId = null
        capabilities.clearConfiguration()
        failSafeStop()
        setStatus("Complete")
    }

    private fun abort(message: String) {
        autoFaulted = true
        finished = true
        cancelActive(message)
        failSafeStop()
        setStatus("Blocked")
        robot.telemetry.putString("ARES/Auto/Error", message)
        runCatching { edu.wpi.first.wpilibj.DriverStation.reportError("ARES auto: $message", false) }
    }

    private fun cancelActive(reason: String) {
        routineManager.cancelAll(reason)
        activeExecutionId = null
        capabilities.clearConfiguration()
    }

    private fun seedPose(pose: Pose2d) {
        sim?.resetPose(pose.x, pose.y, pose.heading.radians)
        robot.swerveDrivetrainIO?.seedPose(pose)
        robot.store.dispatch(
            RobotAction.PoseUpdate(
                xMeters = pose.x,
                yMeters = pose.y,
                headingRadians = pose.heading.radians,
                timestampMs = RobotClock.currentTimeMillis(),
                isReset = true
            )
        )
    }

    private fun validateFieldBounds(entry: AutonomousCatalogEntry) {
        require(GeneratedAresProject.routines.containsKey(entry.routineId)) {
            "Entry '${entry.entryId}' references missing routine '${entry.routineId}'"
        }
        requireFrcRoutinePoseInsideField(capabilities.transform(entry.startingPose), "starting pose")
        val visited = mutableSetOf<String>()
        fun validateRoutine(routineId: String) {
            if (!visited.add(routineId)) return
            val routine = requireNotNull(GeneratedAresProject.routines[routineId]) {
                "Routine '$routineId' does not exist"
            }
            fun validateStep(step: RoutineStep, path: String) {
                step.drive?.target?.let { target ->
                    requireFrcRoutinePoseInsideField(capabilities.transform(target), "$path drive goal")
                }
                step.routineId?.let(::validateRoutine)
                step.deadline?.let { validateStep(it, "$path.deadline") }
                step.children.forEachIndexed { index, child -> validateStep(child, "$path.children[$index]") }
                step.elseChildren.forEachIndexed { index, child ->
                    validateStep(child, "$path.elseChildren[$index]")
                }
            }
            routine.steps.forEachIndexed { index, step -> validateStep(step, "steps[$index]") }
        }
        validateRoutine(entry.routineId)
    }

    private fun failSafeStop() {
        robot.drive.joystickDrive(0.0, 0.0, 0.0, isFieldCentric = false)
        FrcAutoCapabilities.allStopActions().forEach(robot.store::dispatch)
        robot.store.dispatch(SetClimberVoltage(0.0))
        robot.store.dispatch(StopSlamtake())
        robot.safeHardware()
    }

    private fun setStatus(value: String) {
        status = value
        robot.telemetry.putString("ARES/Auto/Selected", selectedAutoId)
        robot.telemetry.putString("ARES/Auto/Status", value)
    }

    internal val isFaultedForTest: Boolean
        get() = autoFaulted
    internal val isFinishedForTest: Boolean
        get() = finished
    internal val selectedAutoForTest: String
        get() = selectedAutoId
    internal val statusForTest: String
        get() = status

    private companion object {
        const val SMART_DASHBOARD_TABLE = "SmartDashboard"
        const val SELECTED_AUTO_ENTRY = "SelectedAuto"
        const val AVAILABLE_AUTOS_ENTRY = "AvailableAutos"

        fun dashboardSelection(): String = runCatching {
            edu.wpi.first.networktables.NetworkTableInstance.getDefault()
                .getTable(SMART_DASHBOARD_TABLE)
                .getEntry(SELECTED_AUTO_ENTRY)
                .getString(GeneratedAresProject.DEFAULT_AUTONOMOUS_ENTRY_ID ?: "do-nothing")
        }.getOrDefault(GeneratedAresProject.DEFAULT_AUTONOMOUS_ENTRY_ID ?: "do-nothing")
    }
}
