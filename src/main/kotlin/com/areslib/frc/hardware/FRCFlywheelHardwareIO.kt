package com.areslib.frc.hardware

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.configs.TalonFXConfiguration

/**
 * Four-motor TalonFX flywheel IO on `CAN2`, arranged as opposed master/follower pairs.
 *
 * Public speed units are RPM; CTRE closed-loop requests use rotations per second at this
 * boundary. [refresh] jointly refreshes both master velocity signals and records whether
 * that observation is trustworthy. Consumers must require [velocityValid] before using
 * cached RPM to authorize feeding. Reverse voltage is disabled by configuration.
 */
class FRCFlywheelHardwareIO(
    private val leftMaster: TalonFX,
    private val leftFollower: TalonFX,
    private val rightMaster: TalonFX,
    private val rightFollower: TalonFX
) : FlywheelIO, FrcMechanismConfigurationStatus {
    override val configurationValid: Boolean
    @Volatile private var cachedVelocityValid = false

    private val velocityRequest = VelocityVoltage(0.0)
    private val voltageRequest = VoltageOut(0.0)

    private val leftMasterVelocity = leftMaster.velocity
    private val rightMasterVelocity = rightMaster.velocity
    private val leftMasterCurrent = leftMaster.statorCurrent
    private val leftFollowerCurrent = leftFollower.statorCurrent
    private val rightMasterCurrent = rightMaster.statorCurrent
    private val rightFollowerCurrent = rightFollower.statorCurrent
    private val leftMasterTemp = leftMaster.deviceTemp
    private val rightMasterTemp = rightMaster.deviceTemp

    init {
        leftFollower.optimizeBusUtilization()
        rightFollower.optimizeBusUtilization()

        setUpdateFrequencies(50.0, leftMasterVelocity, rightMasterVelocity)
        setUpdateFrequencies(20.0, leftMasterCurrent, leftFollowerCurrent, rightMasterCurrent, rightFollowerCurrent)
        setUpdateFrequencies(4.0, leftMasterTemp, rightMasterTemp)

        // Configure followers as opposed to their respective masters
        val leftFollowerConfigured = leftFollower.setControl(
            Follower(leftMaster.deviceID, com.ctre.phoenix6.signals.MotorAlignmentValue.Opposed)
        ).isOK
        val rightFollowerConfigured = rightFollower.setControl(
            Follower(rightMaster.deviceID, com.ctre.phoenix6.signals.MotorAlignmentValue.Opposed)
        ).isOK
        if (!leftFollowerConfigured) reportConfigurationFailure("Flywheel left follower request failed")
        if (!rightFollowerConfigured) reportConfigurationFailure("Flywheel right follower request failed")

        // Enforce exact physical configurations matching SystemConstants.java
        val leftConfigured = listOf(leftMaster, leftFollower).applyMechanismConfigChecked("Flywheel left pair") {
            Slot0.kP = 0.5
            Slot0.kI = 0.0
            Slot0.kD = 0.0
            Slot0.kV = 0.12 // 12.0 / 100.0 (Max speed: 6000 RPM / 60 = 100 RPS)
            Slot0.kS = 0.15 // Conservative static friction compensation, should be tuned via sysid

            MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast
            MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive

            Feedback.SensorToMechanismRatio = 1.0

            Voltage.PeakReverseVoltage = 0.0 // Software lock reversal of flywheel
            CurrentLimits.SupplyCurrentLimitEnable = true
            CurrentLimits.SupplyCurrentLimit = 70.0
            CurrentLimits.StatorCurrentLimitEnable = true
            CurrentLimits.StatorCurrentLimit = 120.0
        }
        
        val rightConfigured = listOf(rightMaster, rightFollower).applyMechanismConfigChecked("Flywheel right pair") {
            Slot0.kP = 0.5
            Slot0.kI = 0.0
            Slot0.kD = 0.0
            Slot0.kV = 0.12 // 12.0 / 100.0 (Max speed: 6000 RPM / 60 = 100 RPS)
            Slot0.kS = 0.15 // Conservative static friction compensation, should be tuned via sysid

            MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast
            MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive

            Feedback.SensorToMechanismRatio = 1.0

            Voltage.PeakReverseVoltage = 0.0 // Software lock reversal of flywheel
            CurrentLimits.SupplyCurrentLimitEnable = true
            CurrentLimits.SupplyCurrentLimit = 70.0
            CurrentLimits.StatorCurrentLimitEnable = true
            CurrentLimits.StatorCurrentLimit = 120.0
        }
        configurationValid = leftFollowerConfigured && rightFollowerConfigured &&
            leftConfigured && rightConfigured
    }



    override fun refresh() {
        cachedVelocityValid = BaseStatusSignal.refreshAll(
            leftMasterVelocity, rightMasterVelocity
        ).isOK
        BaseStatusSignal.refreshAll(
            leftMasterCurrent, leftFollowerCurrent,
            rightMasterCurrent, rightFollowerCurrent,
            leftMasterTemp, rightMasterTemp
        )
    }

    override fun setVelocityRpm(rpm: Double) {
        if (!configurationValid) {
            leftMaster.setControl(voltageRequest.withOutput(0.0))
            rightMaster.setControl(voltageRequest.withOutput(0.0))
            return
        }
        val rps = rpm.takeIf { it.isFinite() && it >= 0.0 }?.div(60.0) ?: 0.0
        leftMaster.setControl(velocityRequest.withVelocity(rps))
        rightMaster.setControl(velocityRequest.withVelocity(rps))
    }

    override fun setAppliedVoltage(volts: Double) {
        val requestedVolts = if (configurationValid) volts else 0.0
        val safeVolts = requestedVolts.takeIf { it.isFinite() }?.coerceIn(0.0, 12.0) ?: 0.0
        leftMaster.setControl(voltageRequest.withOutput(safeVolts))
        rightMaster.setControl(voltageRequest.withOutput(safeVolts))
    }

    override fun configureVelocityController(
        gains: com.areslib.control.tuning.PIDFCoefficients,
        feedforward: com.areslib.control.tuning.SimpleFeedforwardCoeffs
    ) {
        val radiansPerRotation = 2.0 * Math.PI
        val kP = gains.kP * radiansPerRotation
        val kI = gains.kI * radiansPerRotation
        val kD = gains.kD * radiansPerRotation
        val kV = feedforward.kV * radiansPerRotation
        val kA = feedforward.kA * radiansPerRotation
        if (!listOf(kP, kI, kD, kV, kA, feedforward.kS).all { it.isFinite() && it >= 0.0 }) return
        for (motor in listOf(leftMaster, leftFollower, rightMaster, rightFollower)) {
            val config = TalonFXConfiguration()
            if (!motor.configurator.refresh(config).isOK) continue
            config.Slot0.kP = kP
            config.Slot0.kI = kI
            config.Slot0.kD = kD
            config.Slot0.kS = feedforward.kS
            config.Slot0.kV = kV
            config.Slot0.kA = kA
            motor.configurator.apply(config)
        }
    }

    override val velocityRpm: Double
        get() = (leftMasterVelocity.valueAsDouble + rightMasterVelocity.valueAsDouble) / 2.0 * 60.0

    override val velocityValid: Boolean
        get() = cachedVelocityValid

    override val currentAmps: Double
        get() = leftMasterCurrent.valueAsDouble +
                leftFollowerCurrent.valueAsDouble +
                rightMasterCurrent.valueAsDouble +
                rightFollowerCurrent.valueAsDouble

    override val tempCelsius: Double
        get() = Math.max(leftMasterTemp.valueAsDouble, rightMasterTemp.valueAsDouble)
}
