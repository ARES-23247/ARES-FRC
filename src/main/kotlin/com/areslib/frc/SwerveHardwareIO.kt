package com.areslib.frc

import com.areslib.state.DriveState

import com.areslib.hardware.SubsystemIO
import com.areslib.telemetry.ITelemetry

/**
 * Interface representing the hardware input/output for the swerve drivetrain.
 *
 * This allows clean decoupling of the swerve drivetrain logic from Phoenix 6/CTRE hardware
 * JNI classes, facilitating unit testing and simulation.
 */
interface SwerveHardwareIO : SubsystemIO {
    override fun logTelemetry(telemetry: ITelemetry, prefix: String) {
        telemetry.putDoubleArray("$prefix/Currents", currents)
        telemetry.putDoubleArray("$prefix/EncoderPositions", encoderPositions)
    }

    /** Refreshes cached status signals from the hardware. */
    override fun refresh() {}

    /** Reads the drive state from the hardware. */
    fun read(): DriveState

    /** Writes target speeds back to the hardware. */
    fun write(driveState: DriveState)

    /** Gets measured motor supply currents. */
    val currents: DoubleArray
        get() = DoubleArray(4)

    /** Gets measured absolute encoder positions. */
    val encoderPositions: DoubleArray
        get() = DoubleArray(4)

    /** Gets Pigeon2 absolute pitch degrees. */
    val pitchDegrees: Double
        get() = 0.0

    /** Gets Pigeon2 absolute roll degrees. */
    val rollDegrees: Double
        get() = 0.0

    /** Gets measured module linear velocities. */
    val moduleSpeeds: DoubleArray
        get() = DoubleArray(4)

    /** Feeds AprilTag measurements into drivetrain's pose estimator. */
    fun addVisionMeasurement(pose: edu.wpi.first.math.geometry.Pose2d, timestampSeconds: Double) {}

    /** Resets/seeds the underlying pose estimator. */
    fun seedPose(pose: com.areslib.math.Pose2d) {}
}
