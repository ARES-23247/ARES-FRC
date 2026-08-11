package com.areslib.frc

import com.areslib.frc.hardware.FrcMechanismConfigurationStatus
import com.areslib.hardware.drive.SwerveHardwareIO
import com.areslib.math.geometry.Pose2d
import com.areslib.state.DriveState
import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class ARESRobotSafetyBoundaryTest {

    private class FakeSwerveIO : SwerveHardwareIO {
        val encoderPositions = doubleArrayOf(0.1, -0.2, 0.3, -0.4)
        var latencyMs = 0.0
        var encoderValid = true

        override fun read(): DriveState = DriveState()
        override fun write(driveState: DriveState) = Unit
        override fun getEncoderPositions(out: DoubleArray) {
            encoderPositions.copyInto(out)
        }
        override val encoderPositionsValid: Boolean
            get() = encoderValid
        override val signalLatencyMs: Double
            get() = latencyMs
        override fun addVisionMeasurement(pose: Pose2d, timestampSeconds: Double) = Unit
    }

    private class ConfigurationStatus(
        override val configurationValid: Boolean
    ) : FrcMechanismConfigurationStatus

    @Test
    fun `swerve calibration cache requires recent finite plausible four-module sample`() {
        val io = FakeSwerveIO()
        val cache = SwerveOffsetCalibrationSampleCache(maxAgeMs = 100L)
        val output = DoubleArray(4)

        cache.record(io, 1_000L)
        assertTrue(cache.copyFresh(1_100L, output))
        assertArrayEquals(io.encoderPositions, output, 1e-12)
        assertFalse(cache.copyFresh(1_101L, output))

        io.encoderPositions[2] = Double.NaN
        cache.record(io, 1_200L)
        assertFalse(cache.copyFresh(1_200L, output))

        io.encoderPositions[2] = 1.01
        cache.record(io, 1_300L)
        assertFalse(cache.copyFresh(1_300L, output))

        io.encoderPositions[2] = 0.25
        io.latencyMs = 101.0
        cache.record(io, 1_400L)
        assertFalse(cache.copyFresh(1_400L, output))

        io.latencyMs = 0.0
        io.encoderValid = false
        cache.record(io, 1_500L)
        assertFalse(cache.copyFresh(1_500L, output))
    }

    @Test
    fun `mechanism configuration health fails closed on any reporting adapter`() {
        assertTrue(mechanismsConfigured(ConfigurationStatus(true), Any()))
        assertFalse(mechanismsConfigured(ConfigurationStatus(true), ConfigurationStatus(false)))
    }
}
