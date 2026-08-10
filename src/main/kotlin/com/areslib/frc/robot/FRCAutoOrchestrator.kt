package com.areslib.frc.robot

import com.areslib.action.RobotAction
import com.areslib.auto.AresAutoFileLoader
import com.areslib.auto.AutoPose
import com.areslib.auto.AutoRoutine
import com.areslib.auto.AutoRoutineCompiler
import com.areslib.auto.AutoStep
import com.areslib.auto.AutoValidationSeverity
import com.areslib.frc.Dyn4jSimulation
import com.areslib.frc.FrcSwerveRobot
import com.areslib.frc.marvin.SetClimberVoltage
import com.areslib.frc.marvin.MarvinConfig
import com.areslib.frc.marvin.StopSlamtake
import com.areslib.math.coordinate.AllianceMirroring
import com.areslib.math.coordinate.CoordinateTransformers
import com.areslib.math.coordinate.FieldOrigin
import com.areslib.math.coordinate.FieldSymmetry
import com.areslib.math.geometry.Pose2d
import com.areslib.math.geometry.Rotation2d
import com.areslib.pathing.DriveModel
import com.areslib.pathing.JerkLimitedTrajectoryProvider
import com.areslib.pathing.TrajectoryLimits
import com.areslib.pathing.TrajectoryPlanner
import com.areslib.pathing.TrajectoryPreset
import com.areslib.sequencer.Task
import com.areslib.sequencer.TaskExecutor
import com.areslib.sequencer.TaskStateMachine
import com.areslib.sequencer.TaskStatus
import com.areslib.state.Alliance
import com.areslib.util.RobotClock
import java.io.File
import java.io.InputStream
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * Executes one GUI/DSL-authored `.aresauto` during the FRC autonomous period.
 *
 * Autos are authored in Blue-alliance, corner-origin field coordinates. Red execution reflects
 * poses across the alliance-wall axis (`x' = fieldLength - x`) before trajectory generation. The
 * complete document and named-command catalog are compiled before localization is seeded or a task
 * is armed. Missing files, invalid robot footprints, compile errors, and task failures all latch a
 * fail-safe stop.
 */
class FRCAutoOrchestrator @JvmOverloads constructor(
    private val robot: FrcSwerveRobot,
    private val sim: Dyn4jSimulation? = null,
    private val selectionProvider: () -> String = ::dashboardSelection,
    private val directoryProvider: () -> List<File> = ::defaultAutoDirectories,
    private val resourceOpener: ((String) -> InputStream?)? = { resourcePath ->
        FRCAutoOrchestrator::class.java.getResourceAsStream("/deploy/$resourcePath")
    }
) {
    private var executor: TaskExecutor? = null
    private var rootTask: Task? = null
    private var autoFaulted = false
    private var finished = true
    private var selectedAutoId = DEFAULT_AUTO_ID
    private var status = "Idle"

    /** Publishes the offline deploy catalog and initializes the dashboard selection safely. */
    fun publishCatalog() {
        val available = discoverAutos()
        runCatching {
            val table = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
                .getTable(SMART_DASHBOARD_TABLE)
            table.getEntry(SELECTED_AUTO_ENTRY).setDefaultString(DEFAULT_AUTO_ID)
            table.getEntry(AVAILABLE_AUTOS_ENTRY).setStringArray(available.toTypedArray())
        }
        robot.telemetry.putString("ARES/Auto/AvailableDocuments", available.joinToString(","))
    }

    /** Loads, validates, alliance-transforms, compiles, seeds, and arms the selected native auto. */
    fun autonomousInit() {
        cancelExecutor()
        autoFaulted = false
        finished = false
        selectedAutoId = selectionProvider().trim().ifEmpty { DEFAULT_AUTO_ID }

        try {
            val routine = AresAutoFileLoader.load(
                documentId = selectedAutoId,
                directories = directoryProvider(),
                openResource = resourceOpener
            )
            require(routine.documentId == selectedAutoId) {
                "Selected auto '$selectedAutoId' contains document '${routine.documentId}'"
            }
            requireFrcFieldBounds(routine)

            val alliance = robot.store.state.drive.alliance
            val transform = allianceTransform(alliance)
            val compilation = AutoRoutineCompiler(
                trajectoryPlanner = TrajectoryPlanner(listOf(JerkLimitedTrajectoryProvider)),
                follower = com.areslib.pathing.HolonomicPathFollower(robot.drive),
                driveModel = DriveModel.SWERVE,
                limitsForPreset = ::trajectoryLimits,
                poseTransform = transform
            ).compile(routine)
            require(compilation.isSuccess) {
                compilation.issues.joinToString(separator = "; ") { it.message }
            }

            compilation.issues
                .filter { it.severity != AutoValidationSeverity.ERROR }
                .take(3)
                .forEachIndexed { index, warning ->
                    robot.telemetry.putString("ARES/Auto/Warning/${index + 1}", warning.message)
                }

            if (selectedAutoId != DEFAULT_AUTO_ID) {
                seedPose(transform(routine.startingPose))
            }
            val task = requireNotNull(compilation.task) { "Auto compiler produced no executable task" }
            rootTask = task
            executor = TaskExecutor().apply { addTask(task) }
            setStatus("Running")
        } catch (error: Exception) {
            abort("Preflight failed: ${error.message ?: error::class.java.simpleName}")
        }
    }

    /** Advances the shared deterministic task graph once and dispatches all resulting Redux actions. */
    fun autonomousPeriodic() {
        if (finished || autoFaulted) return
        val activeExecutor = executor ?: run {
            abort("Auto executor was not armed")
            return
        }
        val task = rootTask ?: run {
            abort("Compiled auto task is missing")
            return
        }

        try {
            val nowMs = RobotClock.currentTimeMillis()
            activeExecutor.update(robot.store.state, nowMs).forEach(robot.store::dispatch)
            when {
                TaskStateMachine.getStatus(task) == TaskStatus.FAILED ->
                    abort("Task failed: ${task.name}")
                activeExecutor.size == 0 -> complete()
                else -> robot.telemetry.putString(
                    "ARES/Auto/ActiveTask",
                    activeExecutor.activeTaskName ?: task.name
                )
            }
        } catch (error: Exception) {
            abort("Runtime failed: ${error.message ?: error::class.java.simpleName}")
        }
    }

    /** Cancels an active autonomous run and zeros every drivetrain and season output. */
    fun stop() {
        cancelExecutor()
        finished = true
        failSafeStop()
        setStatus("Stopped")
    }

    private fun complete() {
        finished = true
        executor = null
        rootTask = null
        failSafeStop()
        setStatus("Complete")
    }

    private fun abort(message: String) {
        autoFaulted = true
        finished = true
        cancelExecutor()
        failSafeStop()
        setStatus("Blocked")
        robot.telemetry.putString("ARES/Auto/Error", message)
        runCatching { edu.wpi.first.wpilibj.DriverStation.reportError("ARES auto: $message", false) }
    }

    private fun cancelExecutor() {
        executor?.clear(robot.store.state)
        executor = null
        rootTask = null
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

    private fun allianceTransform(alliance: Alliance): (AutoPose) -> Pose2d = { pose ->
        AllianceMirroring.mirror(
            pose = Pose2d(pose.xMeters, pose.yMeters, Rotation2d(pose.headingRadians)),
            alliance = alliance,
            symmetry = FieldSymmetry.MIRRORED,
            fieldLength = CoordinateTransformers.FRC_FIELD_LENGTH,
            fieldWidth = CoordinateTransformers.FRC_FIELD_WIDTH,
            fieldOrigin = FieldOrigin.CORNER
        )
    }

    private fun trajectoryLimits(preset: TrajectoryPreset): TrajectoryLimits {
        val scale = when (preset) {
            TrajectoryPreset.SAFE -> 0.45
            TrajectoryPreset.BALANCED -> 0.70
            TrajectoryPreset.FAST -> 0.90
            TrajectoryPreset.ADAPTIVE -> 0.60
        }
        val maxVelocity = robot.drive.maxSpeedMps * scale
        val maxAcceleration = robot.store.state.tuning.pathAccelerationLimit
            .takeIf { it.isFinite() && it > 0.0 }
            ?.times(scale)
            ?: DEFAULT_ACCELERATION_MPS2 * scale
        return TrajectoryLimits(
            maxVelocityMps = maxVelocity,
            maxAccelerationMps2 = maxAcceleration,
            maxJerkMps3 = maxAcceleration * 4.0,
            maxCentripetalAccelerationMps2 = maxAcceleration * 0.75,
            maxAngularVelocityRps = robot.drive.maxAngularSpeedRadiansPerSecond * scale,
            maxAngularAccelerationRps2 = maxAcceleration / DRIVE_RADIUS_METERS
        )
    }

    private fun requireFrcFieldBounds(routine: AutoRoutine) {
        requirePoseInsideField(routine.startingPose, "starting pose")
        requireStepBounds(routine.steps, "steps")
    }

    private fun requireStepBounds(steps: List<AutoStep>, path: String) {
        steps.forEachIndexed { index, step ->
            step.drive?.target?.let { requirePoseInsideField(it, "$path[$index] drive goal") }
            requireStepBounds(step.children, "$path[$index].children")
        }
    }

    private fun requirePoseInsideField(pose: AutoPose, label: String) {
        val projectedX = abs(cos(pose.headingRadians)) * ROBOT_HALF_LENGTH_METERS +
            abs(sin(pose.headingRadians)) * ROBOT_HALF_WIDTH_METERS
        val projectedY = abs(sin(pose.headingRadians)) * ROBOT_HALF_LENGTH_METERS +
            abs(cos(pose.headingRadians)) * ROBOT_HALF_WIDTH_METERS
        require(
            pose.xMeters in projectedX..(CoordinateTransformers.FRC_FIELD_LENGTH - projectedX) &&
                pose.yMeters in projectedY..(CoordinateTransformers.FRC_FIELD_WIDTH - projectedY)
        ) {
            "$label places the ${ROBOT_LENGTH_METERS} m x $ROBOT_WIDTH_METERS m robot outside the field"
        }
    }

    private fun discoverAutos(): List<String> = buildSet {
        add(DEFAULT_AUTO_ID)
        directoryProvider().forEach { directory ->
            directory.listFiles { file -> file.isFile && file.extension == ARES_AUTO_EXTENSION }
                ?.mapTo(this) { it.nameWithoutExtension }
        }
    }.sorted()

    internal val isFaultedForTest: Boolean
        get() = autoFaulted
    internal val isFinishedForTest: Boolean
        get() = finished
    internal val selectedAutoForTest: String
        get() = selectedAutoId
    internal val statusForTest: String
        get() = status

    private companion object {
        const val DEFAULT_AUTO_ID = "do-nothing"
        const val ARES_AUTO_EXTENSION = "aresauto"
        const val SMART_DASHBOARD_TABLE = "SmartDashboard"
        const val SELECTED_AUTO_ENTRY = "SelectedAuto"
        const val AVAILABLE_AUTOS_ENTRY = "AvailableAutos"
        const val DEFAULT_ACCELERATION_MPS2 = 3.0
        const val DRIVE_RADIUS_METERS = 0.3907
        const val ROBOT_LENGTH_METERS = MarvinConfig.ROBOT_BUMPER_LENGTH_METERS
        const val ROBOT_WIDTH_METERS = MarvinConfig.ROBOT_BUMPER_WIDTH_METERS
        const val ROBOT_HALF_LENGTH_METERS = ROBOT_LENGTH_METERS / 2.0
        const val ROBOT_HALF_WIDTH_METERS = ROBOT_WIDTH_METERS / 2.0

        fun dashboardSelection(): String = runCatching {
            edu.wpi.first.networktables.NetworkTableInstance.getDefault()
                .getTable(SMART_DASHBOARD_TABLE)
                .getEntry(SELECTED_AUTO_ENTRY)
                .getString(DEFAULT_AUTO_ID)
        }.getOrDefault(DEFAULT_AUTO_ID)

        fun defaultAutoDirectories(): List<File> {
            val deploy = runCatching { edu.wpi.first.wpilibj.Filesystem.getDeployDirectory() }.getOrNull()
            return listOfNotNull(
                deploy?.resolve("ares/autos"),
                File("src/main/deploy/ares/autos"),
                File("../ARES-FRC/src/main/deploy/ares/autos")
            ).distinctBy(File::getAbsolutePath)
        }
    }
}
