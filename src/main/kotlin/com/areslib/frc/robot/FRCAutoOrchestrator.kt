package com.areslib.frc.robot

import com.areslib.action.RobotAction
import com.areslib.control.drivetrain.HolonomicDriveController
import com.areslib.control.feedback.PIDController
import com.areslib.frc.FrcSwerveRobot
import com.areslib.frc.Dyn4jSimulation
import com.areslib.frc.marvin.MarvinIntakeSubsystem
import com.areslib.frc.marvin.MarvinShooterSubsystem
import com.areslib.pathing.Path
import com.areslib.pathing.MutablePathPoint
import com.areslib.frc.aresAlliance
import com.areslib.frc.marvin.marvin
/**
 * Documentation for FRCAutoOrchestrator
 */

class FRCAutoOrchestrator(
    private val robot: FrcSwerveRobot,
    private val sim: Dyn4jSimulation,
    private val marvinShooter: MarvinShooterSubsystem,
    private val marvinIntake: MarvinIntakeSubsystem
) {
    private var activePath: Path? = null
    private var autoStartTime = 0.0
    private var autoDistance = 0.0
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

                val startPoint = activePath?.points?.firstOrNull()
                if (startPoint != null && isFirstPath) {
                    val autoTable = edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("Auto")
                    val startX = autoTable.getEntry("InitialPoseX").getDouble(startPoint.pose.x)
                    val startY = autoTable.getEntry("InitialPoseY").getDouble(startPoint.pose.y)
                    val startHeading = autoTable.getEntry("InitialPoseHeading").getDouble(startPoint.pose.heading.radians)
                    sim.resetPose(startX, startY, startHeading)

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
        commandWaitStartTime = 0.0
        autoStartTime = com.areslib.util.RobotClock.currentTimeMillis() / 1000.0
        val estimator = robot.store.state.drive.poseEstimator
        lastOdomX = estimator.estimatedPoseX
        lastOdomY = estimator.estimatedPoseY
        autoDistance = 0.0
        previousDistance = 0.0
        lastLoopTime = autoStartTime
    }
    /**
     * Documentation for autonomousPeriodic
     */

    fun autonomousPeriodic() {
        try {
            /**
             * Documentation for path
             */
            val path = activePath ?: return
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
            autoDistance = kotlin.math.min(autoDistance, totalLength)

            path.sampleAtDistance(autoDistance, scratchPathPoint)

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
                val discretizedSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds.discretize(
                    edu.wpi.first.math.kinematics.ChassisSpeeds(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond
                    ),
                    dt
                )
                robot.drive.joystickDrive(
                    discretizedSpeeds.vxMetersPerSecond,
                    discretizedSpeeds.vyMetersPerSecond,
                    discretizedSpeeds.omegaRadiansPerSecond,
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
                    targetVelocityMps = scratchPathPoint.velocityMps,
                    pathTangentRadians = scratchPathPoint.tangentRadians,
                    dtSeconds = dt
                )

                // HolonomicDriveController.calculateDirect() already returns robot-relative speeds
                val discretizedSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds.discretize(
                    edu.wpi.first.math.kinematics.ChassisSpeeds(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond
                    ),
                    dt
                )
                robot.drive.joystickDrive(
                    discretizedSpeeds.vxMetersPerSecond,
                    discretizedSpeeds.vyMetersPerSecond,
                    discretizedSpeeds.omegaRadiansPerSecond,
                    isFieldCentric = false
                )
            }

            // Event markers
            for (i in 0 until path.events.size) {
                /**
                 * Documentation for event
                 */
                val event = path.events[i]
                
                if (autoDistance >= event.triggerDistanceMeters && i !in triggeredEvents) {
                    var eventCompleted = true
                    when (event.eventName) {
                        "FlywheelOn" -> marvinShooter.spinUp(4000.0)
                        "IntakeDeploy" -> {
                            marvinIntake.deploy()
                            marvinIntake.setRollerSpeed(15.0)
                        }
                        "FeederShoot" -> {
                            val marvinState = robot.store.state.superstructure.marvin
                            val isRpmAligned = marvinState.flywheel.targetVelocityRpm > 100.0 && kotlin.math.abs(marvinState.flywheel.velocityRpm - marvinState.flywheel.targetVelocityRpm) < 150.0
                            if (isRpmAligned) {
                                marvinShooter.shoot()
                                isWaitingForCommand = false
                                commandWaitStartTime = 0.0
                            } else {
                                if (!isWaitingForCommand) {
                                    commandWaitStartTime = currentTime
                                }
                                if (currentTime - commandWaitStartTime > 2.0) {
                                    isWaitingForCommand = false
                                    eventCompleted = true
                                    commandWaitStartTime = 0.0
                                } else {
                                    isWaitingForCommand = true
                                    eventCompleted = false
                                }
                            }
                        }
                    }
                    if (eventCompleted) {
                        println("AUTO EVENT TRIGGERED: ${event.eventName} at ${event.triggerDistanceMeters}m")
                        robot.telemetry.putString("Robot/ActiveEvent", event.eventName)
                        triggeredEvents.add(i)
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
            if (!isWaitingForCommand) {
                val minSearch = (autoDistance - 0.5).coerceAtLeast(0.0)
                val maxSearch = (autoDistance + 0.5).coerceAtMost(path.points.lastOrNull()?.distanceMeters ?: 0.0)
                autoDistance = path.findClosestDistance(estimator.estimatedPoseX, estimator.estimatedPoseY, minSearch, maxSearch)
            }
        } catch (e: Exception) {
            edu.wpi.first.wpilibj.DriverStation.reportError("Exception in autonomousPeriodic: ${e.message}", false)
            robot.safeHardware()
        }
    }
}
