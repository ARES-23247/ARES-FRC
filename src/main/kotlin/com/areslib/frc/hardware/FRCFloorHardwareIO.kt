package com.areslib.frc.hardware

import com.areslib.frc.hardware.FloorIO
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX

/**
 * Open-loop floor-roller TalonFX IO for CAN ID 16 on `CAN2`.
 *
 * Redux expresses the requested roller speed in RPS, but [setAppliedVoltage] is the
 * hardware boundary because this mechanism intentionally has no velocity loop. Velocity
 * and current getters return signals cached by [refresh].
 */
class FRCFloorHardwareIO(
    private val motor: TalonFX
) : FloorIO, FrcMechanismConfigurationStatus {
    override val configurationValid: Boolean

    private val voltageRequest = VoltageOut(0.0)

    private val floorVelocity = motor.velocity
    private val floorCurrent = motor.statorCurrent

    init {
        motor.optimizeBusUtilization()
        setUpdateFrequencies(20.0, floorVelocity)
        setUpdateFrequencies(10.0, floorCurrent)

        configurationValid = listOf(motor).applyMechanismConfigChecked("Floor roller") {
            MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast
            MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive
            Feedback.SensorToMechanismRatio = 1.0

            CurrentLimits.SupplyCurrentLimitEnable = true
            CurrentLimits.SupplyCurrentLimit = 60.0
            CurrentLimits.StatorCurrentLimitEnable = true
            CurrentLimits.StatorCurrentLimit = 100.0
        }
    }

    override fun refresh() {
        floorVelocity.refresh()
        floorCurrent.refresh()
    }

    /**
     * Drives the floor rollers open-loop via raw voltage. This is deliberate: the
     * floor is a high-speed intake roller governed by voltage feed-forward only
     * (FLOOR_KV * rps * brownoutScale), so no TalonFX Slot0 PID gains are configured.
     */
    override fun setAppliedVoltage(volts: Double) {
        val requestedVolts = if (configurationValid) volts else 0.0
        motor.setControl(voltageRequest.withOutput(
            requestedVolts.takeIf { it.isFinite() }?.coerceIn(-12.0, 12.0) ?: 0.0
        ))
    }

    override val velocityRps: Double
        get() = floorVelocity.valueAsDouble

    override val currentAmps: Double
        get() = floorCurrent.valueAsDouble
}
