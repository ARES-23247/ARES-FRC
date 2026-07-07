package com.areslib.frc

import com.areslib.state.DriveState
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.kinematics.ChassisSpeeds

/**
 * Hardware IO bridge for an FRC Phoenix 6 Swerve Drive.
 * 
 * This class abstracts the highly-optimized CTRE SwerveDrivetrain (which runs internally
 * at 250Hz on the CAN FD bus) into the pure mathematical ARESLib Redux architecture.
 */
class FRCSwerveHardwareIO(private val drivetrain: SwerveDrivetrain<*, *, *>) : SwerveHardwareIO {

    // CTRE Swerve request object we will mutate every loop
    private val robotSpeedsRequest = SwerveRequest.ApplyRobotSpeeds()
    private val scratchSpeeds = ChassisSpeeds()

    private val currentDraw1 = drivetrain.getModule(0).driveMotor.supplyCurrent
    private val currentDraw2 = drivetrain.getModule(1).driveMotor.supplyCurrent
    private val currentDraw3 = drivetrain.getModule(2).driveMotor.supplyCurrent
    private val currentDraw4 = drivetrain.getModule(3).driveMotor.supplyCurrent

    private val absEnc1 = (drivetrain.getModule(0).encoder as com.ctre.phoenix6.hardware.CANcoder).absolutePosition
    private val absEnc2 = (drivetrain.getModule(1).encoder as com.ctre.phoenix6.hardware.CANcoder).absolutePosition
    private val absEnc3 = (drivetrain.getModule(2).encoder as com.ctre.phoenix6.hardware.CANcoder).absolutePosition
    private val absEnc4 = (drivetrain.getModule(3).encoder as com.ctre.phoenix6.hardware.CANcoder).absolutePosition

    private val pigeon = com.ctre.phoenix6.hardware.Pigeon2(9, com.ctre.phoenix6.CANBus("CAN2"))
    private val pitchSignal = pigeon.pitch
    private val rollSignal = pigeon.roll

    init {
        for (i in 0..3) {
            drivetrain.getModule(i).driveMotor.supplyCurrent.setUpdateFrequency(20.0)
            (drivetrain.getModule(i).encoder as com.ctre.phoenix6.hardware.CANcoder).absolutePosition.setUpdateFrequency(50.0)
        }
        pitchSignal.setUpdateFrequency(20.0)
        rollSignal.setUpdateFrequency(20.0)
    }

    override fun refresh() {
        BaseStatusSignal.refreshAll(
            currentDraw1, currentDraw2, currentDraw3, currentDraw4,
            absEnc1, absEnc2, absEnc3, absEnc4,
            pitchSignal, rollSignal
        )
    }

    override fun safe() {
        write(DriveState())
    }

    override fun getCurrents(out: DoubleArray) {
        out[0] = currentDraw1.valueAsDouble
        out[1] = currentDraw2.valueAsDouble
        out[2] = currentDraw3.valueAsDouble
        out[3] = currentDraw4.valueAsDouble
    }

    override fun getEncoderPositions(out: DoubleArray) {
        out[0] = absEnc1.valueAsDouble
        out[1] = absEnc2.valueAsDouble
        out[2] = absEnc3.valueAsDouble
        out[3] = absEnc4.valueAsDouble
    }

    override val pitchDegrees: Double
        get() = pitchSignal.valueAsDouble

    override val rollDegrees: Double
        get() = rollSignal.valueAsDouble

    override fun getModuleSpeeds(out: DoubleArray) {
        out[0] = drivetrain.state.ModuleStates[0].speedMetersPerSecond
        out[1] = drivetrain.state.ModuleStates[1].speedMetersPerSecond
        out[2] = drivetrain.state.ModuleStates[2].speedMetersPerSecond
        out[3] = drivetrain.state.ModuleStates[3].speedMetersPerSecond
    }

    /**
     * Reads the 250Hz synchronized pose from the CTRE drivetrain and maps it
     * into the Redux DriveState object for the next calculation cycle.
     */
    override fun read(): DriveState {
        val driveStateObj = drivetrain.state
        val pose = driveStateObj.Pose

        return DriveState(
            // We read the actual measured velocities from the drivetrain state
            xVelocityMetersPerSecond = driveStateObj.Speeds.vxMetersPerSecond,
            yVelocityMetersPerSecond = driveStateObj.Speeds.vyMetersPerSecond,
            angularVelocityRadiansPerSecond = driveStateObj.Speeds.omegaRadiansPerSecond,
            
            // Map the internal 250Hz odometry to the Redux state
            odometryX = pose.x,
            odometryY = pose.y,
            odometryHeading = pose.rotation.radians
        )
    }

    /**
     * Applies the macro-level ChassisSpeeds computed by the Redux reducers 
     * back to the CTRE SwerveDrivetrain. The CTRE firmware handles the inverse 
     * kinematics and closed-loop motor control internally.
     */
    override fun write(driveState: DriveState) {
        if (driveState.isFieldCentric) {
            val c = Math.cos(driveState.odometryHeading)
            val s = Math.sin(driveState.odometryHeading)
            scratchSpeeds.vxMetersPerSecond = driveState.xVelocityMetersPerSecond * c + driveState.yVelocityMetersPerSecond * s
            scratchSpeeds.vyMetersPerSecond = -driveState.xVelocityMetersPerSecond * s + driveState.yVelocityMetersPerSecond * c
            scratchSpeeds.omegaRadiansPerSecond = driveState.angularVelocityRadiansPerSecond
        } else {
            scratchSpeeds.vxMetersPerSecond = driveState.xVelocityMetersPerSecond
            scratchSpeeds.vyMetersPerSecond = driveState.yVelocityMetersPerSecond
            scratchSpeeds.omegaRadiansPerSecond = driveState.angularVelocityRadiansPerSecond
        }
        
        // Pass the speeds to the CTRE API
        drivetrain.setControl(robotSpeedsRequest.withSpeeds(scratchSpeeds))
    }

    /**
     * Feeds AprilTag vision measurements into the CTRE SwerveDrivetrain's internal EKF.
     */
    override fun addVisionMeasurement(pose: edu.wpi.first.math.geometry.Pose2d, timestampSeconds: Double) {
        drivetrain.addVisionMeasurement(pose, timestampSeconds)
    }

    /**
     * Resets/seeds the CTRE SwerveDrivetrain internal odometry pose.
     */
    override fun seedPose(pose: com.areslib.math.Pose2d) {
        val wpiPose = edu.wpi.first.math.geometry.Pose2d(
            pose.x,
            pose.y,
            edu.wpi.first.math.geometry.Rotation2d.fromRadians(pose.heading.radians)
        )
        drivetrain.resetPose(wpiPose)
    }
}
