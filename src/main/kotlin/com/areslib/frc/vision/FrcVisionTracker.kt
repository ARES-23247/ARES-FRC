package com.areslib.frc.vision

import com.areslib.action.RobotAction
import com.areslib.hardware.vision.VisionIO
import com.areslib.hardware.vision.VisionIOInputs
import com.areslib.subsystem.Store
import com.areslib.telemetry.RobotStatusTracker

class FrcVisionTracker(
    private val store: Store,
    val visionIO: VisionIO?,
    private val swerveIO: com.areslib.frc.SwerveHardwareIO?,
    private val isSimulation: Boolean
) {
    val visionInputs = VisionIOInputs()

    /**
     * Polls the vision inputs, passes vehicle state to the camera (for MegaTag2),
     * and streams valid measurements back to the drivetrain Kalman filter and Redux store.
     */
    fun update(timestamp: Long) {
        visionIO?.let { io ->
            val drive = store.state.drive
            io.setOrientation(
                yawDegrees = Math.toDegrees(drive.odometryHeading),
                yawRateDegPerSec = Math.toDegrees(drive.angularVelocityRadiansPerSecond),
                pitchDegrees = drive.pitchDegrees,
                pitchRateDegPerSec = 0.0,
                rollDegrees = drive.rollDegrees,
                rollRateDegPerSec = 0.0,
                linearVelocityMps = Math.hypot(drive.xVelocityMetersPerSecond, drive.yVelocityMetersPerSecond)
            )
            io.updateInputs(visionInputs)
            if (visionInputs.measurements.isNotEmpty()) {
                val measurement = visionInputs.measurements[0]
                if (!isSimulation && swerveIO != null) {
                    try {
                        val wpiPose = edu.wpi.first.math.geometry.Pose2d(
                            measurement.targetPose.translation.x,
                            measurement.targetPose.translation.y,
                            edu.wpi.first.math.geometry.Rotation2d.fromRadians(measurement.targetPose.rotation.z)
                        )
                        val latencyMs = timestamp - measurement.timestampMs
                        val timestampSec = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - (latencyMs / 1000.0)
                        swerveIO.addVisionMeasurement(wpiPose, timestampSec)
                    } catch (e: Throwable) {
                        System.err.println("FrcSwerveRobot: Failed to feed vision to SwerveDrivetrain: ${e.message}")
                    }
                }
                store.dispatch(RobotAction.VisionMeasurementsReceived(
                    visionInputs.measurements,
                    timestamp,
                    null
                ))
            }
            RobotStatusTracker.visionConnected = visionInputs.isConnected
        } ?: run {
            RobotStatusTracker.visionConnected = false
        }
    }
}
