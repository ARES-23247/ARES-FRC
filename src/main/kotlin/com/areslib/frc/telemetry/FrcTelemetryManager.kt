package com.areslib.frc.telemetry

import com.areslib.control.BrownoutGuard
import com.areslib.frc.telemetry.MarvinStatePublisher
import com.areslib.hardware.*
import com.areslib.state.RobotState
import com.areslib.subsystem.Store
import com.areslib.telemetry.*

class FrcTelemetryManager(baseTelemetry: ITelemetry, private val store: Store) : AutoCloseable {

    // Unified telemetry pipeline: base telemetry → CSV wrapper → publisher
    val dataLoggingTelemetry = DataLoggingTelemetry(baseTelemetry)
    val publisher = ARESNetworkStatePublisher(dataLoggingTelemetry)

    // Pre-allocated buffers to prevent high-frequency GC allocations in update loop
    private val covarianceDiagonals = DoubleArray(3)
    private val swerveStates = DoubleArray(8)

    /**
     * Publishes core robot state, Marvin XIX specific sub-states, and AdvantageScope 3D visualization topics.
     */
    fun publish(gamepad1: GamepadState? = null, gamepad2: GamepadState? = null) {
        val state = store.state
        publisher.publish(state, gamepad1, gamepad2)

        // Publish Marvin XIX specific sub-states
        MarvinStatePublisher.publish(state, dataLoggingTelemetry)

        // Publish EKF covariance diagonals
        val cov = state.drive.poseEstimator.covariance
        covarianceDiagonals[0] = cov.m00
        covarianceDiagonals[1] = cov.m11
        covarianceDiagonals[2] = cov.m22
        dataLoggingTelemetry.putDoubleArray("Robot/Odometry/Covariance", covarianceDiagonals)

        // Publish 3D robot pose (quaternion format for AdvantageScope)
        dataLoggingTelemetry.logPose3d(
            "Robot/Pose3d",
            state.drive.odometryX,
            state.drive.odometryY,
            state.drive.odometryHeading
        )

        // Publish swerve module states
        val vx = state.drive.xVelocityMetersPerSecond
        val vy = state.drive.yVelocityMetersPerSecond
        val omega = state.drive.angularVelocityRadiansPerSecond
        for (i in 0..3) {
            val wvx = vx - omega * SWERVE_OFFSETS[i].second
            val wvy = vy + omega * SWERVE_OFFSETS[i].first
            swerveStates[i * 2] = Math.atan2(wvy, wvx)
            swerveStates[i * 2 + 1] = Math.hypot(wvx, wvy)
        }
        dataLoggingTelemetry.putDoubleArray("Robot/SwerveStates", swerveStates)
    }

    /**
     * Publishes brownout telemetry.
     */
    fun logBrownout(brownoutGuard: BrownoutGuard, batteryVoltage: Double) {
        dataLoggingTelemetry.logBrownout(brownoutGuard, batteryVoltage)
    }



    override fun close() {
        dataLoggingTelemetry.close()
    }

    companion object {
        private val SWERVE_OFFSETS = arrayOf(
            Pair(0.35, 0.35), Pair(0.35, -0.35),
            Pair(-0.35, 0.35), Pair(-0.35, -0.35)
        )
    }
}
