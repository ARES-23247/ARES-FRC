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

    // Climber scaling: 1 mechanism rotation is treated as the extension unit
    private val rotationsPerMeter = 1.0

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
            Slot0.kP = 1.0
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
        BaseStatusSignal.refreshAll(climberPosition, climberCurrent)
    }

    override fun setTargetExtension(meters: Double) {
        /**
         * Documentation for targetRotations
         */
        val targetRotations = meters * rotationsPerMeter
        motor.setControl(positionRequest.withPosition(targetRotations))
    }

    override fun setAppliedVoltage(volts: Double) {
        motor.setControl(voltageRequest.withOutput(volts))
    }

    override val extensionMeters: Double
        get() = climberPosition.valueAsDouble / rotationsPerMeter

    override val currentAmps: Double
        get() = climberCurrent.valueAsDouble
}
