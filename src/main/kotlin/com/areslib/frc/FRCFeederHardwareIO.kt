package com.areslib.frc

import com.areslib.hardware.FeederIO
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
        feederCurrent.setUpdateFrequency(10.0)

        val config = com.ctre.phoenix6.configs.TalonFXConfiguration()
        config.Slot0.kP = 1.0
        config.Slot0.kI = 0.0
        config.Slot0.kD = 0.0
        config.Slot0.kV = 0.48 // 12.0 / 25.0 (Max speed: 6000 RPM / 4 = 1500 RPM = 25 RPS)

        config.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast
        config.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
        config.Feedback.SensorToMechanismRatio = 4.0 // 4:1 feeder gear reduction

        config.CurrentLimits.SupplyCurrentLimitEnable = true
        config.CurrentLimits.SupplyCurrentLimit = 60.0
        config.CurrentLimits.StatorCurrentLimitEnable = true
        config.CurrentLimits.StatorCurrentLimit = 100.0
        motor.configurator.apply(config)
    }

    override fun refresh() {
        BaseStatusSignal.refreshAll(feederCurrent)
    }

    override fun setAppliedVoltage(volts: Double) {
        motor.setControl(voltageRequest.withOutput(volts))
    }

    override val isBeamBroken: Boolean
        get() = false

    override val currentAmps: Double
        get() = feederCurrent.valueAsDouble
}
