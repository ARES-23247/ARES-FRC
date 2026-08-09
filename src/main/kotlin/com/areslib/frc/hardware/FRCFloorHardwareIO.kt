package com.areslib.frc.hardware

import com.areslib.frc.hardware.FloorIO
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX

/**
 * Concrete implementation of FloorIO utilizing a CTRE TalonFX motor
 * on ID 16 on the "CAN2" high-speed bus.
 */
class FRCFloorHardwareIO(
    private val motor: TalonFX
) : FloorIO {

    private val voltageRequest = VoltageOut(0.0)

    private val floorVelocity = motor.velocity
    private val floorCurrent = motor.statorCurrent

    init {
        motor.optimizeBusUtilization()
        setUpdateFrequencies(20.0, floorVelocity)
        setUpdateFrequencies(10.0, floorCurrent)

        listOf(motor).applyConfig {
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
        motor.setControl(voltageRequest.withOutput(volts))
    }

    override val velocityRps: Double
        get() = floorVelocity.valueAsDouble

    override val currentAmps: Double
        get() = floorCurrent.valueAsDouble
}
