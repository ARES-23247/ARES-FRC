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
    
    private val driveController = HolonomicDriveController(
        PIDController(4.0, 0.0, 0.1),
        PIDController(4.0, 0.0, 0.1),
        PIDController(3.0, 0.0, 0.0)
    )

    private val targetPoseScratch = DoubleArray(3)
    private val scratchPathPoint = MutablePathPoint()
    /**
     * Documentation for autonomousInit
     */

    fun autonomousInit() {
        try {
            /**
             * Documentation for path
             */
            var path = com.areslib.frc.PathLoader.loadPath("SimPath")
            
            path = com.areslib.math.coordinate.AllianceMirroring.mirror(
                path,
                aresAlliance,
                com.areslib.math.coordinate.FieldSymmetry.MIRRORED,
                fieldLength = com.areslib.math.coordinate.CoordinateTransformers.FRC_FIELD_LENGTH,
                fieldWidth = com.areslib.math.coordinate.CoordinateTransformers.FRC_FIELD_WIDTH
            )
            activePath = path
            /**
             * Documentation for startPoint
             */

            val startPoint = activePath?.points?.firstOrNull()
            if (startPoint != null) {
                sim.resetPose(startPoint.pose.x, startPoint.pose.y, startPoint.pose.heading.radians)
                
                // Seed physical CTRE swerve drivetrain to prevent reset desync step jump
                robot.swerveDrivetrainIO?.seedPose(
                    com.areslib.math.geometry.Pose2d(
                        startPoint.pose.x,
                        startPoint.pose.y,
                        com.areslib.math.geometry.Rotation2d(startPoint.pose.heading.radians)
                    )
                )

                robot.store.dispatch(RobotAction.PoseUpdate(
                    xMeters = startPoint.pose.x,
                    yMeters = startPoint.pose.y,
                    headingRadians = startPoint.pose.heading.radians,
                    timestampMs = com.areslib.util.RobotClock.currentTimeMillis(),
                    isReset = true
                ))
            }
        } catch (e: Exception) {
            println("ERROR: Failed to load autonomous path SimPath: ${e.message}")
            activePath = null
        }
        autoStartTime = com.areslib.util.RobotClock.currentTimeMillis() / 1000.0
        autoDistance = 0.0
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
            val dt = 0.02
            /**
             * Documentation for currentPose
             */

            val estimator = robot.store.state.drive.poseEstimator

            path.sampleAtDistance(autoDistance, scratchPathPoint)

            val speeds = driveController.calculateDirect(
                currentX = estimator.estimatedPoseX,
                currentY = estimator.estimatedPoseY,
                currentHeadingRad = estimator.estimatedPoseHeading,
                targetX = scratchPathPoint.x,
                targetY = scratchPathPoint.y,
                targetHeadingRad = scratchPathPoint.headingRad,
                targetVelocityMps = scratchPathPoint.velocityMps,
                dtSeconds = dt
            )

            // HolonomicDriveController.calculateDirect() already returns robot-relative speeds
            robot.drive.joystickDrive(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                isFieldCentric = false
            )

            // Event markers
            for (i in 0 until path.events.size) {
                /**
                 * Documentation for event
                 */
                val event = path.events[i]
                /**
                 * Documentation for nextDistance
                 */
                val nextDistance = autoDistance + scratchPathPoint.velocityMps * dt
                if (event.triggerDistanceMeters in autoDistance..nextDistance) {
                    println("AUTO EVENT TRIGGERED: ${event.eventName} at ${event.triggerDistanceMeters}m")
                    robot.telemetry.putString("Robot/ActiveEvent", event.eventName)
                    when (event.eventName) {
                        "FlywheelOn" -> marvinShooter.spinUp(4000.0)
                        "IntakeDeploy" -> {
                            marvinIntake.deploy()
                            marvinIntake.setRollerSpeed(15.0)
                        }
                        "FeederShoot" -> marvinShooter.shoot()
                    }
                }
            }

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

            autoDistance += scratchPathPoint.velocityMps * dt
        } catch (e: Throwable) {
            System.err.println("ARESRobot: Exception in autonomousPeriodic: ${e.message}")
            e.printStackTrace()
            robot.safeHardware()
        }
    }
}
