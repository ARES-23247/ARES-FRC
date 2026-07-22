package com.areslib.frc.hardware

import com.areslib.frc.hardware.IntakeIO
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX

/**
 * Concrete implementation of IntakeIO utilizing CTRE TalonFX motors for pivot
 * and rollers, tracking position via the internal motor encoder.
 *
 * Designed for Marvin 19's CAN2 bus and 4:1 gear feedback ratio (no CANcoder).
 */
class FRCIntakeHardwareIO(
    private val pivotMotor: TalonFX,
    private val rollerMotor: TalonFX
) : IntakeIO {

    private val positionRequest = PositionVoltage(0.0)
    private val voltageRequest = VoltageOut(0.0)

    private val pivotPosition = pivotMotor.position
    private val pivotCurrent = pivotMotor.statorCurrent
    private val rollerCurrent = rollerMotor.statorCurrent

    init {
        pivotMotor.optimizeBusUtilization()
        rollerMotor.optimizeBusUtilization()

        pivotPosition.setUpdateFrequency(50.0)
        pivotCurrent.setUpdateFrequency(10.0)
        rollerCurrent.setUpdateFrequency(10.0)

        listOf(pivotMotor).applyConfig {
            Slot0.kP = 1.0
            Slot0.kI = 0.0
            Slot0.kD = 0.0
            Slot0.kV = 0.38247 // 12.0 / 31.375 (Max speed: 7530 RPM / 4 = 1882.5 RPM = 31.375 RPS)

            MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake
            MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            Feedback.SensorToMechanismRatio = 4.0 // 4:1 pivot gear reduction

            CurrentLimits.SupplyCurrentLimitEnable = true
            CurrentLimits.SupplyCurrentLimit = 40.0
            CurrentLimits.StatorCurrentLimitEnable = true
            CurrentLimits.StatorCurrentLimit = 80.0
        }

        listOf(rollerMotor).applyConfig {
            Slot0.kP = 0.5
            Slot0.kI = 2.0
            Slot0.kD = 0.0
            Slot0.kV = 0.0956 // 12.0 / 125.5 (Max speed: 7530 RPM = 125.5 RPS)

            MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast
            MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            Feedback.SensorToMechanismRatio = 1.0

            CurrentLimits.SupplyCurrentLimitEnable = true
            CurrentLimits.SupplyCurrentLimit = 60.0
            CurrentLimits.StatorCurrentLimitEnable = true
            CurrentLimits.StatorCurrentLimit = 100.0
        }
    }

    override fun refresh() {
        BaseStatusSignal.refreshAll(pivotPosition, pivotCurrent, rollerCurrent)
    }

    override fun setPivotAngle(degrees: Double) {
        // Convert degrees to mechanism rotations (1 degree = (1.0 / 360.0) rotations)
        // Feedback.SensorToMechanismRatio handles the internal 4:1 scaling in TalonFX
        /**
         * Documentation for rotations
         */
        val rotations = degrees / 360.0
        pivotMotor.setControl(positionRequest.withPosition(rotations))
    }

    override fun setPivotVoltage(volts: Double) {
        pivotMotor.setControl(voltageRequest.withOutput(volts))
    }

    override fun setRollerVoltage(volts: Double) {
        rollerMotor.setControl(voltageRequest.withOutput(volts))
    }

    override val pivotAngleDegrees: Double
        get() = pivotPosition.valueAsDouble * 360.0

    override val pivotCurrentAmps: Double
        get() = pivotCurrent.valueAsDouble

    override val rollerCurrentAmps: Double
        get() = rollerCurrent.valueAsDouble
}
