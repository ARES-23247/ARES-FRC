package com.areslib.frc.hardware

import com.areslib.frc.marvin.MarvinConfig
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
            SoftwareLimitSwitch.ForwardSoftLimitThreshold = MarvinConfig.cowlMaxRotations
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
        cowlPosition.refresh()
        cowlCurrent.refresh()
    }

    override fun setTargetAngle(rotations: Double) {
        // Use target cowl angle directly
        motor.setControl(positionRequest.withPosition(rotations))
    }

    override fun setTargetAngle(rotations: Double, maxEffortScale: Double) {
        val effortScale = maxEffortScale.coerceIn(0.0, 1.0)
        if (effortScale >= FULL_EFFORT_THRESHOLD) {
            setTargetAngle(rotations)
            return
        }
        val error = rotations - angleRotations
        val staticVolts = when {
            error > POSITION_EPSILON_ROTATIONS -> POSITION_KS_VOLTS
            error < -POSITION_EPSILON_ROTATIONS -> -POSITION_KS_VOLTS
            else -> 0.0
        }
        val maxVolts = NOMINAL_VOLTAGE * effortScale
        setAppliedVoltage((POSITION_KP_VOLTS_PER_ROTATION * error + staticVolts).coerceIn(-maxVolts, maxVolts))
    }

    override fun setAppliedVoltage(volts: Double) {
        motor.setControl(voltageRequest.withOutput(volts))
    }

    override val angleRotations: Double
        get() = cowlPosition.valueAsDouble

    override val currentAmps: Double
        get() = cowlCurrent.valueAsDouble

    private companion object {
        const val POSITION_KP_VOLTS_PER_ROTATION = 20.0
        const val POSITION_KS_VOLTS = 2.0
        const val POSITION_EPSILON_ROTATIONS = 0.002
        const val NOMINAL_VOLTAGE = 12.0
        const val FULL_EFFORT_THRESHOLD = 0.999
    }
}
