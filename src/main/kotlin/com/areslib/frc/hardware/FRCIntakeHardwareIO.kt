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

        // Configure Pivot Motor with exact specs from SystemConstants.java
        /**
         * Documentation for pivotConfig
         */
        val pivotConfig = com.ctre.phoenix6.configs.TalonFXConfiguration()
        pivotConfig.Slot0.kP = 1.0
        pivotConfig.Slot0.kI = 0.0
        pivotConfig.Slot0.kD = 0.0
        pivotConfig.Slot0.kV = 0.38247 // 12.0 / 31.375 (Max speed: 7530 RPM / 4 = 1882.5 RPM = 31.375 RPS)

        pivotConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake
        pivotConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
        pivotConfig.Feedback.SensorToMechanismRatio = 4.0 // 4:1 pivot gear reduction

        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 40.0
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true
        pivotConfig.CurrentLimits.StatorCurrentLimit = 80.0
        pivotMotor.configurator.apply(pivotConfig)

        // Configure Roller Motor with exact specs from SystemConstants.java
        /**
         * Documentation for rollerConfig
         */
        val rollerConfig = com.ctre.phoenix6.configs.TalonFXConfiguration()
        rollerConfig.Slot0.kP = 0.5
        rollerConfig.Slot0.kI = 2.0
        rollerConfig.Slot0.kD = 0.0
        rollerConfig.Slot0.kV = 0.0956 // 12.0 / 125.5 (Max speed: 7530 RPM = 125.5 RPS)

        rollerConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast
        rollerConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
        rollerConfig.Feedback.SensorToMechanismRatio = 1.0

        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 60.0
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true
        rollerConfig.CurrentLimits.StatorCurrentLimit = 100.0
        rollerMotor.configurator.apply(rollerConfig)
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
