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

    override val currents: DoubleArray
        get() = doubleArrayOf(
            currentDraw1.valueAsDouble,
            currentDraw2.valueAsDouble,
            currentDraw3.valueAsDouble,
            currentDraw4.valueAsDouble
        )

    override val encoderPositions: DoubleArray
        get() = doubleArrayOf(
            absEnc1.valueAsDouble,
            absEnc2.valueAsDouble,
            absEnc3.valueAsDouble,
            absEnc4.valueAsDouble
        )

    override val pitchDegrees: Double
        get() = pitchSignal.valueAsDouble

    override val rollDegrees: Double
        get() = rollSignal.valueAsDouble

    override val moduleSpeeds: DoubleArray
        get() = doubleArrayOf(
            drivetrain.state.ModuleStates[0].speedMetersPerSecond,
            drivetrain.state.ModuleStates[1].speedMetersPerSecond,
            drivetrain.state.ModuleStates[2].speedMetersPerSecond,
            drivetrain.state.ModuleStates[3].speedMetersPerSecond
        )

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
        val speeds = if (driveState.isFieldCentric) {
            val heading = edu.wpi.first.math.geometry.Rotation2d.fromRadians(driveState.odometryHeading)
            ChassisSpeeds.fromFieldRelativeSpeeds(
                driveState.xVelocityMetersPerSecond,
                driveState.yVelocityMetersPerSecond,
                driveState.angularVelocityRadiansPerSecond,
                heading
            )
        } else {
            ChassisSpeeds(
                driveState.xVelocityMetersPerSecond,
                driveState.yVelocityMetersPerSecond,
                driveState.angularVelocityRadiansPerSecond
            )
        }
        
        // Pass the speeds to the CTRE API
        drivetrain.setControl(robotSpeedsRequest.withSpeeds(speeds))
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
