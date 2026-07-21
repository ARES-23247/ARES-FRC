package com.areslib.frc.hardware

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
        setUpdateFrequencies(50.0, cowlPosition)
        setUpdateFrequencies(10.0, cowlCurrent)

        listOf(motor).applyConfig {
            // Neutral mode and inversions
            MotorOutput.NeutralMode = NeutralModeValue.Brake
            MotorOutput.Inverted = InvertedValue.Clockwise_Positive

            // Gearing and sensor ratio
            Feedback.SensorToMechanismRatio = 1.0

            // Software soft limits
            SoftwareLimitSwitch.ForwardSoftLimitEnable = true
            SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.80
            SoftwareLimitSwitch.ReverseSoftLimitEnable = true
            SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0

            // Position closed-loop PID gains
            Slot0.kP = 20.0
            Slot0.kI = 0.0
            Slot0.kD = 0.0
            Slot0.kS = 2.0

            // Current limits
            CurrentLimits.SupplyCurrentLimit = 30.0
            CurrentLimits.SupplyCurrentLimitEnable = true
            CurrentLimits.StatorCurrentLimit = 50.0
            CurrentLimits.StatorCurrentLimitEnable = true
        }
    }

    override fun refresh() {
        BaseStatusSignal.refreshAll(cowlPosition, cowlCurrent)
    }

    override fun setTargetAngle(rotations: Double) {
        // Use target cowl angle directly
        motor.setControl(positionRequest.withPosition(rotations))
    }

    override fun setAppliedVoltage(volts: Double) {
        motor.setControl(voltageRequest.withOutput(volts))
    }

    override val angleRotations: Double
        get() = cowlPosition.valueAsDouble

    override val currentAmps: Double
        get() = cowlCurrent.valueAsDouble
}
