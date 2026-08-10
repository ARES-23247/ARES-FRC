package com.areslib.frc.hardware

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX

/**
 * TalonFX IO for the Marvin XIX intake pivot and roller.
 *
 * The public pivot contract is degrees; commands are converted to mechanism rotations
 * after CTRE's configured 4:1 sensor ratio. Roller commands use RPS. [refresh] is the
 * sole sensor-read phase, so position/current getters remain cached and CAN-free.
 */
class FRCIntakeHardwareIO(
    private val pivotMotor: TalonFX,
    private val rollerMotor: TalonFX
) : IntakeIO {
    @Volatile private var cachedPivotAngleValid = false

    private val positionRequest = PositionVoltage(0.0)
    private val voltageRequest = VoltageOut(0.0)
    private val velocityRequest = com.ctre.phoenix6.controls.VelocityVoltage(0.0)

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
            Slot0.kP = 24.0
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

            // Software soft limits. Pivot travel is 0.0 (stowed) to ~0.25 mechanism rotations
            // (90° deploy; setPivotAngle commands degrees/360). Forward threshold of 0.30 gives
            // a 0.05-rotation margin above the full-deploy command, mirroring the cowl/climber.
            SoftwareLimitSwitch.ForwardSoftLimitEnable = true
            SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.30
            SoftwareLimitSwitch.ReverseSoftLimitEnable = true
            SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0
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
            CurrentLimits.SupplyCurrentLimit = 30.0
            CurrentLimits.StatorCurrentLimitEnable = true
            CurrentLimits.StatorCurrentLimit = 40.0
        }
    }

    override fun refresh() {
        cachedPivotAngleValid = BaseStatusSignal.refreshAll(pivotPosition).isOK &&
            pivotPosition.valueAsDouble.isFinite()
        BaseStatusSignal.refreshAll(pivotCurrent, rollerCurrent)
    }

    override fun setPivotAngle(degrees: Double) {
        // Convert degrees to mechanism rotations (1 degree = (1.0 / 360.0) rotations)
        // Feedback.SensorToMechanismRatio handles the internal 4:1 scaling in TalonFX
        val safeDegrees = degrees.takeIf { it.isFinite() }?.coerceIn(
            com.areslib.frc.marvin.MarvinConfig.MechanismLimits.intakeStowedDegrees,
            com.areslib.frc.marvin.MarvinConfig.MechanismLimits.intakeDeployedDegrees
        ) ?: com.areslib.frc.marvin.MarvinConfig.MechanismLimits.intakeStowedDegrees
        val rotations = safeDegrees / 360.0
        pivotMotor.setControl(positionRequest.withPosition(rotations))
    }

    override fun setPivotAngle(degrees: Double, maxEffortScale: Double) {
        val effortScale = maxEffortScale.takeIf { it.isFinite() }?.coerceIn(0.0, 1.0) ?: 0.0
        if (effortScale >= FULL_EFFORT_THRESHOLD) {
            setPivotAngle(degrees)
            return
        }
        val safeDegrees = degrees.takeIf { it.isFinite() }?.coerceIn(
            com.areslib.frc.marvin.MarvinConfig.MechanismLimits.intakeStowedDegrees,
            com.areslib.frc.marvin.MarvinConfig.MechanismLimits.intakeDeployedDegrees
        ) ?: com.areslib.frc.marvin.MarvinConfig.MechanismLimits.intakeStowedDegrees
        val targetRotations = safeDegrees / 360.0
        val errorRotations = targetRotations - pivotPosition.valueAsDouble
        val maxVolts = NOMINAL_VOLTAGE * effortScale
        setPivotVoltage((POSITION_KP_VOLTS_PER_ROTATION * errorRotations).coerceIn(-maxVolts, maxVolts))
    }

    override fun setPivotVoltage(volts: Double) {
        pivotMotor.setControl(voltageRequest.withOutput(volts.takeIf { it.isFinite() }?.coerceIn(-12.0, 12.0) ?: 0.0))
    }

    override fun setRollerVoltage(volts: Double) {
        rollerMotor.setControl(voltageRequest.withOutput(volts.takeIf { it.isFinite() }?.coerceIn(-12.0, 12.0) ?: 0.0))
    }

    override fun setRollerVelocityRps(rps: Double) {
        rollerMotor.setControl(velocityRequest.withVelocity(rps.takeIf { it.isFinite() } ?: 0.0))
    }

    override val pivotAngleDegrees: Double
        get() = pivotPosition.valueAsDouble * 360.0

    override val pivotAngleValid: Boolean
        get() = cachedPivotAngleValid

    override val pivotCurrentAmps: Double
        get() = pivotCurrent.valueAsDouble

    override val rollerCurrentAmps: Double
        get() = rollerCurrent.valueAsDouble

    private companion object {
        const val POSITION_KP_VOLTS_PER_ROTATION = 24.0
        const val NOMINAL_VOLTAGE = 12.0
        const val FULL_EFFORT_THRESHOLD = 0.999
    }
}
