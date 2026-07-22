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
            Slot0.kP = 0.5
            Slot0.kI = 2.0
            Slot0.kD = 0.0
            Slot0.kV = 0.0956 // 12.0 / 125.5 (Max speed: 7530 RPM = 125.5 RPS)

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
        BaseStatusSignal.refreshAll(floorVelocity, floorCurrent)
    }

    override fun setAppliedVoltage(volts: Double) {
        motor.setControl(voltageRequest.withOutput(volts))
    }

    override val velocityRps: Double
        get() = floorVelocity.valueAsDouble

    override val currentAmps: Double
        get() = floorCurrent.valueAsDouble
}
