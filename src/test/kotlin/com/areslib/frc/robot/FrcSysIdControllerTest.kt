package com.areslib.frc.robot

import com.areslib.control.tuning.PIDFCoefficients
import com.areslib.control.tuning.SimpleFeedforwardCoeffs
import com.areslib.hardware.actuator.FlywheelIO
import com.areslib.state.MechanismTuningState
import com.areslib.state.RobotState
import com.areslib.state.SubsystemTuningState
import com.areslib.state.TuningState
import com.areslib.telemetry.ITelemetry
import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class FrcSysIdControllerTest {
    @Test
    fun `flywheel dynamic routine applies voltage and emits canonical sample`() {
        val telemetry = FakeTelemetry().apply { strings["SysId/Command"] = "START_FLYWHEEL_DYNAMIC" }
        val flywheel = FakeFlywheel(measuredRpm = 1200.0)
        val controller = FrcSysIdController(telemetry, flywheel)

        controller.update(1_000L, RobotState(), enabledForTuning = true)

        assertEquals(6.0, flywheel.lastAppliedVoltage, 1e-9)
        assertEquals("DYNAMIC", telemetry.strings["SysId/Status"])
        assertArrayEquals(
            doubleArrayOf(1000.0, 6.0, 0.0, 1200.0 * 2.0 * Math.PI / 60.0, 0.0),
            telemetry.arrays["SysId/Data"],
            1e-9
        )
    }

    @Test
    fun `routine is rejected outside test mode`() {
        val telemetry = FakeTelemetry().apply { strings["SysId/Command"] = "START_FLYWHEEL_DYNAMIC" }
        val flywheel = FakeFlywheel(measuredRpm = 1000.0)

        FrcSysIdController(telemetry, flywheel).update(1_000L, RobotState(), enabledForTuning = false)

        assertEquals(0.0, flywheel.lastAppliedVoltage, 1e-9)
        assertEquals("FRC_SYSID_REQUIRES_TEST_MODE", telemetry.strings["SysId/Error"])
        assertEquals("NONE", telemetry.strings["SysId/Status"])
    }

    @Test
    fun `identified gains are applied to flywheel controller`() {
        val telemetry = FakeTelemetry()
        val flywheel = FakeFlywheel(measuredRpm = 0.0)
        val gains = PIDFCoefficients(0.4, 0.01, 0.03, 0.0)
        val feedforward = SimpleFeedforwardCoeffs(0.2, 0.12, 0.01)
        val state = RobotState(
            tuning = TuningState(
                subsystem = SubsystemTuningState(
                    flywheel = MechanismTuningState(feedforward, gains)
                )
            )
        )

        FrcSysIdController(telemetry, flywheel).update(1_000L, state, enabledForTuning = true)

        assertEquals(gains, flywheel.configuredGains)
        assertEquals(feedforward, flywheel.configuredFeedforward)
    }

    private class FakeFlywheel(private var measuredRpm: Double) : FlywheelIO {
        var lastAppliedVoltage = 0.0
        var configuredGains: PIDFCoefficients? = null
        var configuredFeedforward: SimpleFeedforwardCoeffs? = null

        override val velocityRpm: Double get() = measuredRpm
        override val velocityValid: Boolean = true
        override fun setVelocityRpm(rpm: Double) { measuredRpm = rpm }
        override fun setAppliedVoltage(volts: Double) { lastAppliedVoltage = volts }
        override fun configureVelocityController(gains: PIDFCoefficients, feedforward: SimpleFeedforwardCoeffs) {
            configuredGains = gains
            configuredFeedforward = feedforward
        }
    }

    private class FakeTelemetry : ITelemetry {
        val strings = mutableMapOf<String, String>()
        val arrays = mutableMapOf<String, DoubleArray>()
        override fun putNumber(key: String, value: Double) = Unit
        override fun putBoolean(key: String, value: Boolean) = Unit
        override fun putString(key: String, value: String) { strings[key] = value }
        override fun putDoubleArray(key: String, value: DoubleArray) { arrays[key] = value.copyOf() }
        override fun getNumber(key: String, defaultValue: Double) = defaultValue
        override fun getBoolean(key: String, defaultValue: Boolean) = defaultValue
        override fun getString(key: String, defaultValue: String) = strings[key] ?: defaultValue
    }
}
