package com.areslib.frc.robot

import com.areslib.action.RobotAction
import com.areslib.control.drivetrain.HolonomicDriveController
import com.areslib.control.feedback.PIDController
import com.areslib.frc.FrcSwerveRobot
import com.areslib.frc.Dyn4jSimulation
import com.areslib.frc.marvin.MarvinIntakeSubsystem
import com.areslib.frc.marvin.MarvinShooterSubsystem
import com.areslib.frc.marvin.MarvinConfig
import com.areslib.frc.marvin.SetFeederSpeed
import com.areslib.pathing.Path
import com.areslib.pathing.MutablePathPoint
import com.areslib.frc.aresAlliance
import com.areslib.frc.marvin.marvin
/**
 * Documentation for FRCAutoOrchestrator
 */

class FRCAutoOrchestrator(
    private val robot: FrcSwerveRobot,
    private val sim: Dyn4jSimulation?,
    private val marvinShooter: MarvinShooterSubsystem,
    private val marvinIntake: MarvinIntakeSubsystem
) {
    private var activePath: Path? = null
    private var autoStartTime = 0.0
    private var autoDistance = 0.0
    private var actualPathDistance = 0.0
    private var profileElapsedSeconds = 0.0
    private var profilePointTimes = DoubleArray(0)
    private var profileSegmentIndex = 0
    private var waitingHoldDistance = Double.NaN
    private var autoFaulted = false
    private var lastLoopTime = 0.0
    private var lastOdomX = 0.0
    private var lastOdomY = 0.0
    
    private val triggeredEvents = mutableSetOf<Int>()
    private var isWaitingForCommand = false
    private var isFirstPath = true
    private var commandWaitStartTime = 0.0
    private var previousDistance = 0.0
    
    private val driveController = HolonomicDriveController(
        PIDController(5.0, 0.0, 0.5),
        PIDController(5.0, 0.0, 0.5),
        PIDController(4.0, 0.0, 0.4).apply { enableContinuousInput(-Math.PI, Math.PI) }
    )

    private val targetPoseScratch = DoubleArray(3)
    private val scratchPathPoint = MutablePathPoint()
    /**
     * Documentation for autonomousInit
     */

    fun autonomousInit() {
        isFirstPath = true
        var pathName = ""
        try {
            pathName = edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("SelectedPath").getString("SimPath")
            if (edu.wpi.first.wpilibj.RobotBase.isReal() && pathName == "SimPath") {
                edu.wpi.first.wpilibj.DriverStation.reportError("No auto path selected (default SimPath)", true)
                activePath = Path(emptyList())
            } else {
                var path = com.areslib.frc.PathLoader.loadPath(pathName)

                path = com.areslib.math.coordinate.AllianceMirroring.mirror(
                    path,
                    aresAlliance,
                    com.areslib.math.coordinate.FieldSymmetry.MIRRORED,
                    fieldLength = com.areslib.math.coordinate.CoordinateTransformers.FRC_FIELD_LENGTH,
                    fieldWidth = com.areslib.math.coordinate.CoordinateTransformers.FRC_FIELD_WIDTH
                )
                activePath = path
                buildProfileTimeline(path)

                val startPoint = activePath?.points?.firstOrNull()
                if (startPoint != null && isFirstPath) {
                    val autoTable = edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("Auto")
                    val startX = autoTable.getEntry("InitialPoseX").getDouble(startPoint.pose.x)
                    val startY = autoTable.getEntry("InitialPoseY").getDouble(startPoint.pose.y)
                    val startHeading = autoTable.getEntry("InitialPoseHeading").getDouble(startPoint.pose.heading.radians)
                    sim?.resetPose(startX, startY, startHeading)

                    // Seed physical CTRE swerve drivetrain to prevent reset desync step jump
                    robot.swerveDrivetrainIO?.seedPose(
                        com.areslib.math.geometry.Pose2d(
                            startX,
                            startY,
                            com.areslib.math.geometry.Rotation2d(startHeading)
                        )
                    )

                    robot.store.dispatch(RobotAction.PoseUpdate(
                        xMeters = startX,
                        yMeters = startY,
                        headingRadians = startHeading,
                        timestampMs = com.areslib.util.RobotClock.currentTimeMillis(),
                        isReset = true
                    ))
                    isFirstPath = false
                }
            }
        } catch (e: Exception) {
            println("ERROR: Failed to load autonomous path: ${e.message}")
            activePath = Path(emptyList())
            edu.wpi.first.wpilibj.DriverStation.reportError("Missing auto path file: $pathName", false)
        }
        triggeredEvents.clear()
        isWaitingForCommand = false
        autoFaulted = false
        commandWaitStartTime = 0.0
        autoStartTime = com.areslib.util.RobotClock.currentTimeMillis() / 1000.0
        val estimator = robot.store.state.drive.poseEstimator
        lastOdomX = estimator.estimatedPoseX
        lastOdomY = estimator.estimatedPoseY
        autoDistance = 0.0
        actualPathDistance = 0.0
        profileElapsedSeconds = 0.0
        profileSegmentIndex = 0
        waitingHoldDistance = Double.NaN
        previousDistance = 0.0
        lastLoopTime = autoStartTime
    }
    /**
     * Documentation for autonomousPeriodic
     */

    fun autonomousPeriodic() {
        if (autoFaulted) return
        try {
            /**
             * Documentation for path
             */
            val path = activePath ?: return
            if (path.points.isEmpty()) {
                autoFaulted = true
                failSafeStop()
                return
            }
            /**
             * Documentation for dt
             */
            val currentTime = com.areslib.util.RobotClock.currentTimeMillis() / 1000.0
            if (lastLoopTime <= 0.0) {
                lastLoopTime = currentTime
                return
            }
            val dt = (currentTime - lastLoopTime).coerceIn(0.005, 0.1)
            lastLoopTime = currentTime
            /**
             * Documentation for currentPose
             */

            val estimator = robot.store.state.drive.poseEstimator

            val totalLength = path.points.lastOrNull()?.distanceMeters ?: 0.0
            if (!isWaitingForCommand) profileElapsedSeconds += dt
            autoDistance = if (isWaitingForCommand && waitingHoldDistance.isFinite()) {
                waitingHoldDistance
            } else {
                distanceAtProfileTime(path, profileElapsedSeconds)
            }.coerceIn(0.0, totalLength)

            path.sampleAtDistance(autoDistance, scratchPathPoint)

            val minSearch = (actualPathDistance - 0.75).coerceAtLeast(0.0)
            val maxSearch = (actualPathDistance + 0.75).coerceAtMost(totalLength)
            actualPathDistance = path.findClosestDistance(
                estimator.estimatedPoseX,
                estimator.estimatedPoseY,
                minSearch,
                maxSearch
            )

            if (autoDistance >= totalLength) {
                val speeds = driveController.calculateDirect(
                    currentX = estimator.estimatedPoseX,
                    currentY = estimator.estimatedPoseY,
                    currentHeadingRad = estimator.estimatedPoseHeading,
                    targetX = scratchPathPoint.x,
                    targetY = scratchPathPoint.y,
                    targetHeadingRad = scratchPathPoint.headingRad,
                    targetVelocityMps = 0.0,
                    pathTangentRadians = scratchPathPoint.tangentRadians,
                    dtSeconds = dt
                )
                robot.drive.joystickDrive(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond,
                    isFieldCentric = false
                )
            } else {
                val speeds = driveController.calculateDirect(
                    currentX = estimator.estimatedPoseX,
                    currentY = estimator.estimatedPoseY,
                    currentHeadingRad = estimator.estimatedPoseHeading,
                    targetX = scratchPathPoint.x,
                    targetY = scratchPathPoint.y,
                    targetHeadingRad = scratchPathPoint.headingRad,
                    targetVelocityMps = if (isWaitingForCommand) 0.0 else scratchPathPoint.velocityMps,
                    pathTangentRadians = scratchPathPoint.tangentRadians,
                    dtSeconds = dt
                )

                robot.drive.joystickDrive(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond,
                    isFieldCentric = false
                )
            }

            // Event markers
            for (i in 0 until path.events.size) {
                /**
                 * Documentation for event
                 */
                val event = path.events[i]
                
                if (actualPathDistance >= event.triggerDistanceMeters && i !in triggeredEvents) {
                    val wasWaiting = isWaitingForCommand
                    val eventCompleted = handleEvent(event.eventName, currentTime)
                    if (eventCompleted) {
                        println("AUTO EVENT TRIGGERED: ${event.eventName} at ${event.triggerDistanceMeters}m")
                        robot.telemetry.putString("Robot/ActiveEvent", event.eventName)
                        triggeredEvents.add(i)
                        waitingHoldDistance = Double.NaN
                    } else if (!wasWaiting && isWaitingForCommand) {
                        waitingHoldDistance = event.triggerDistanceMeters.coerceIn(0.0, totalLength)
                        profileElapsedSeconds = profileTimeAtDistance(path, waitingHoldDistance)
                        autoDistance = waitingHoldDistance
                    }
                }
            }
            previousDistance = autoDistance

            // Trajectory telemetry
            targetPoseScratch[0] = scratchPathPoint.x
            targetPoseScratch[1] = scratchPathPoint.y
            targetPoseScratch[2] = scratchPathPoint.headingRad
            robot.telemetry.putDoubleArray("Robot/TargetPose", targetPoseScratch)
            /**
             * Documentation for dx
             */
            val dx = scratchPathPoint.x - estimator.estimatedPoseX
            val dy = scratchPathPoint.y - estimator.estimatedPoseY
            robot.telemetry.putNumber("Robot/TrajectoryError", kotlin.math.hypot(dx, dy))

            val actualDx = estimator.estimatedPoseX - lastOdomX
            val actualDy = estimator.estimatedPoseY - lastOdomY
            lastOdomX = estimator.estimatedPoseX
            lastOdomY = estimator.estimatedPoseY
        } catch (e: Exception) {
            edu.wpi.first.wpilibj.DriverStation.reportError("Exception in autonomousPeriodic: ${e.message}", false)
            autoFaulted = true
            failSafeStop()
        }
    }

    /**
     * Dispatches the actions for a single autonomous path event by name.
     *
     * Extracted from autonomousPeriodic so the production event-dispatch path is
     * unit-testable without running the trajectory follower.
     *
     * @return true if the event completed, false if it must keep waiting.
     */
    internal fun handleEvent(eventName: String, currentTimeSeconds: Double): Boolean {
        var eventCompleted = true
        when (eventName) {
            "FlywheelOn" -> marvinShooter.spinUp(4000.0)
            "IntakeDeploy" -> {
                marvinIntake.deploy()
                marvinIntake.setRollerSpeed(15.0)
            }
            "FeederShoot" -> {
                val marvinState = robot.store.state.superstructure.marvin
                val isRpmAligned = marvinState.flywheel.velocityValid &&
                    marvinState.flywheel.targetVelocityRpm > 100.0 &&
                    kotlin.math.abs(marvinState.flywheel.velocityRpm - marvinState.flywheel.targetVelocityRpm) < 150.0
                if (isRpmAligned) {
                    marvinShooter.shoot()
                    robot.store.dispatch(SetFeederSpeed(MarvinConfig.FEEDER_SHOOT_SPEED_RPS, com.areslib.util.RobotClock.currentTimeMillis()))
                    isWaitingForCommand = false
                    commandWaitStartTime = 0.0
                } else {
                    if (!isWaitingForCommand) {
                        commandWaitStartTime = currentTimeSeconds
                    }
                    if (currentTimeSeconds - commandWaitStartTime > 2.0) {
                        isWaitingForCommand = false
                        commandWaitStartTime = 0.0
                    } else {
                        isWaitingForCommand = true
                        eventCompleted = false
                    }
                }
            }
        }
        return eventCompleted
    }

    private fun buildProfileTimeline(path: Path) {
        val points = path.points
        profilePointTimes = DoubleArray(points.size)
        var cumulativeSeconds = 0.0
        for (i in 1 until points.size) {
            val distance = (points[i].distanceMeters - points[i - 1].distanceMeters).coerceAtLeast(0.0)
            val velocitySum = (points[i - 1].velocityMps + points[i].velocityMps).coerceAtLeast(0.0)
            val segmentSeconds = if (velocitySum > MIN_PROFILE_VELOCITY_MPS) {
                2.0 * distance / velocitySum
            } else {
                distance / MIN_PROFILE_VELOCITY_MPS
            }
            cumulativeSeconds += segmentSeconds
            profilePointTimes[i] = cumulativeSeconds
        }
    }

    private fun distanceAtProfileTime(path: Path, elapsedSeconds: Double): Double {
        val points = path.points
        if (points.isEmpty()) return 0.0
        if (points.size == 1 || profilePointTimes.size != points.size) return points.first().distanceMeters
        if (elapsedSeconds >= profilePointTimes.last()) return points.last().distanceMeters

        while (profileSegmentIndex + 1 < profilePointTimes.size &&
            elapsedSeconds > profilePointTimes[profileSegmentIndex + 1]) {
            profileSegmentIndex++
        }
        while (profileSegmentIndex > 0 && elapsedSeconds < profilePointTimes[profileSegmentIndex]) {
            profileSegmentIndex--
        }

        val start = points[profileSegmentIndex]
        val end = points[profileSegmentIndex + 1]
        val segmentStartTime = profilePointTimes[profileSegmentIndex]
        val duration = profilePointTimes[profileSegmentIndex + 1] - segmentStartTime
        if (duration <= 1e-9) return end.distanceMeters
        val localTime = (elapsedSeconds - segmentStartTime).coerceIn(0.0, duration)
        val distanceDelta = end.distanceMeters - start.distanceMeters
        val traveled = if (start.velocityMps + end.velocityMps > MIN_PROFILE_VELOCITY_MPS) {
            val acceleration = (end.velocityMps - start.velocityMps) / duration
            start.velocityMps * localTime + 0.5 * acceleration * localTime * localTime
        } else {
            distanceDelta * (localTime / duration)
        }
        return (start.distanceMeters + traveled).coerceIn(start.distanceMeters, end.distanceMeters)
    }

    private fun profileTimeAtDistance(path: Path, distanceMeters: Double): Double {
        val points = path.points
        if (points.size < 2 || profilePointTimes.size != points.size) return 0.0
        for (i in 0 until points.lastIndex) {
            val start = points[i]
            val end = points[i + 1]
            if (distanceMeters <= end.distanceMeters) {
                val distanceDelta = end.distanceMeters - start.distanceMeters
                val segmentDuration = profilePointTimes[i + 1] - profilePointTimes[i]
                if (distanceDelta <= 1e-9 || segmentDuration <= 1e-9) return profilePointTimes[i]
                val localDistance = (distanceMeters - start.distanceMeters).coerceIn(0.0, distanceDelta)
                val localTime = if (start.velocityMps + end.velocityMps > MIN_PROFILE_VELOCITY_MPS) {
                    val acceleration = (end.velocityMps - start.velocityMps) / segmentDuration
                    if (kotlin.math.abs(acceleration) <= 1e-9) {
                        if (start.velocityMps > 1e-9) localDistance / start.velocityMps else 0.0
                    } else {
                        val discriminant = (start.velocityMps * start.velocityMps +
                            2.0 * acceleration * localDistance).coerceAtLeast(0.0)
                        (-start.velocityMps + kotlin.math.sqrt(discriminant)) / acceleration
                    }
                } else {
                    segmentDuration * (localDistance / distanceDelta)
                }.coerceIn(0.0, segmentDuration)
                return profilePointTimes[i] + localTime
            }
        }
        return profilePointTimes.last()
    }

    private fun failSafeStop() {
        robot.drive.joystickDrive(0.0, 0.0, 0.0, isFieldCentric = false)
        marvinShooter.stop()
        marvinIntake.setRollerSpeed(0.0)
        robot.store.dispatch(com.areslib.frc.marvin.SetClimberVoltage(0.0))
        robot.store.dispatch(com.areslib.frc.marvin.StopSlamtake())
        robot.safeHardware()
    }

    internal val targetDistanceMetersForTest: Double get() = autoDistance
    internal val actualDistanceMetersForTest: Double get() = actualPathDistance
    internal val isFaultedForTest: Boolean get() = autoFaulted

    private companion object {
        const val MIN_PROFILE_VELOCITY_MPS = 0.10
    }
}
