package com.areslib.frc.hardware

import com.areslib.frc.hardware.CowlIO
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

/**
 * Concrete implementation of CowlIO utilizing a single CTRE TalonFX motor
 * to actuate the adjustable hood angle.
 * 
 * Configured in mechanism rotations directly (0.50 to 1.75 mechanism rotations),
 * matching Marvin 19 system constants and SOTM interpolations.
 */
class FRCCowlHardwareIO(
    private val motor: TalonFX
) : CowlIO {

    private val positionRequest = PositionVoltage(0.0)
    private val voltageRequest = VoltageOut(0.0)

    private val cowlPosition = motor.position
    private val cowlCurrent = motor.statorCurrent

    init {
        motor.optimizeBusUtilization()
        cowlPosition.setUpdateFrequency(50.0)
        cowlCurrent.setUpdateFrequency(10.0)

        val config = TalonFXConfiguration()
        
        // Neutral mode and inversions
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

        // Gearing and sensor ratio
        config.Feedback.SensorToMechanismRatio = 1.0

        // Software soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.80
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.1

        // Position closed-loop PID gains
        config.Slot0.kP = 20.0
        config.Slot0.kI = 0.0
        config.Slot0.kD = 0.0
        config.Slot0.kS = 2.0

        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = 30.0
        config.CurrentLimits.SupplyCurrentLimitEnable = true
        config.CurrentLimits.StatorCurrentLimit = 50.0
        config.CurrentLimits.StatorCurrentLimitEnable = true

        motor.configurator.apply(config)
    }

    override fun refresh() {
        BaseStatusSignal.refreshAll(cowlPosition, cowlCurrent)
    }

    override fun setTargetAngle(degrees: Double) {
        // Use target cowl angle directly
        val rotations = degrees
        motor.setControl(positionRequest.withPosition(rotations))
    }

    override fun setAppliedVoltage(volts: Double) {
        motor.setControl(voltageRequest.withOutput(volts))
    }

    override val angleDegrees: Double
        get() = cowlPosition.valueAsDouble

    override val currentAmps: Double
        get() = cowlCurrent.valueAsDouble
}
