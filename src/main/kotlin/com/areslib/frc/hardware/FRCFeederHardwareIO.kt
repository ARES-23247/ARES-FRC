package com.areslib.frc.hardware

import com.areslib.frc.hardware.FeederIO
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX

/**
 * Concrete implementation of FeederIO utilizing a CTRE TalonFX motor on CAN2.
 * Note: Marvin 19 does not have a physical beam break sensor.
 */
class FRCFeederHardwareIO(
    private val motor: TalonFX
) : FeederIO {

    private val voltageRequest = VoltageOut(0.0)

    private val feederCurrent = motor.statorCurrent

    init {
        motor.optimizeBusUtilization()
        setUpdateFrequencies(10.0, feederCurrent)

        listOf(motor).applyConfig {
            Slot0.kP = 1.0
            Slot0.kI = 0.0
            Slot0.kD = 0.0
            Slot0.kV = 0.48 // 12.0 / 25.0 (Max speed: 6000 RPM / 4 = 1500 RPM = 25 RPS)

            MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast
            MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            Feedback.SensorToMechanismRatio = 4.0 // 4:1 feeder gear reduction

            CurrentLimits.SupplyCurrentLimitEnable = true
            CurrentLimits.SupplyCurrentLimit = 60.0
            CurrentLimits.StatorCurrentLimitEnable = true
            CurrentLimits.StatorCurrentLimit = 100.0
        }
    }

    override fun refresh() {
        feederCurrent.refresh()
    }

    override fun setAppliedVoltage(volts: Double) {
        motor.setControl(voltageRequest.withOutput(volts))
    }

    override val isBeamBroken: Boolean
        get() = false

    override val currentAmps: Double
        get() = feederCurrent.valueAsDouble
}
