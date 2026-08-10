package com.areslib.frc.hardware

import com.areslib.frc.hardware.ClimberIO
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

/**
 * Concrete implementation of ClimberIO utilizing a CTRE TalonFX motor
 * on ID 19 on the "CAN2" high-speed bus, with configured soft limits.
 */
class FRCClimberHardwareIO(
    private val motor: TalonFX
) : ClimberIO {

    private val positionRequest = PositionVoltage(0.0)
    private val voltageRequest = VoltageOut(0.0)

    private val climberPosition = motor.position
    private val climberCurrent = motor.statorCurrent

    init {
        motor.optimizeBusUtilization()
        setUpdateFrequencies(50.0, climberPosition)
        setUpdateFrequencies(10.0, climberCurrent)

        listOf(motor).applyConfig {
            // Neutral mode and inversions
            MotorOutput.NeutralMode = NeutralModeValue.Brake
            MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
            
            // Gearing / Sensor scaling
            Feedback.SensorToMechanismRatio = 80.0

            // Supply and Stator current limits matching SystemConstants.java
            CurrentLimits.SupplyCurrentLimit = 70.0
            CurrentLimits.SupplyCurrentLimitEnable = true
            CurrentLimits.StatorCurrentLimit = 120.0
            CurrentLimits.StatorCurrentLimitEnable = true

            // Position closed-loop PID/feedforward gains
            Slot0.kP = 12.0
            Slot0.kI = 0.0
            Slot0.kD = 0.0
            Slot0.kV = 9.6 // 12.0 / 1.25 RPS (Max speed: 6000 RPM / 80 = 75 RPM = 1.25 RPS)

            // Software soft limits
            SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.73
            SoftwareLimitSwitch.ForwardSoftLimitEnable = true
            SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0
            SoftwareLimitSwitch.ReverseSoftLimitEnable = true
        }
    }

    override fun refresh() {
        climberPosition.refresh()
        climberCurrent.refresh()
    }

    override fun setTargetPositionRotations(rotations: Double) {
        motor.setControl(positionRequest.withPosition(rotations))
    }

    override fun setTargetPositionRotations(rotations: Double, maxEffortScale: Double) {
        val effortScale = maxEffortScale.coerceIn(0.0, 1.0)
        if (effortScale >= FULL_EFFORT_THRESHOLD) {
            setTargetPositionRotations(rotations)
            return
        }
        val error = rotations - positionRotations
        val maxVolts = NOMINAL_VOLTAGE * effortScale
        setAppliedVoltage((POSITION_KP_VOLTS_PER_ROTATION * error).coerceIn(-maxVolts, maxVolts))
    }

    override fun setAppliedVoltage(volts: Double) {
        motor.setControl(voltageRequest.withOutput(volts))
    }

    override val positionRotations: Double
        get() = climberPosition.valueAsDouble

    override val currentAmps: Double
        get() = climberCurrent.valueAsDouble

    private companion object {
        const val POSITION_KP_VOLTS_PER_ROTATION = 12.0
        const val NOMINAL_VOLTAGE = 12.0
        const val FULL_EFFORT_THRESHOLD = 0.999
    }
}
