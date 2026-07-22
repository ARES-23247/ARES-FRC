package com.areslib.frc.hardware

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX

/**
 * Concrete implementation of FlywheelIO utilizing 4 physical CTRE TalonFX motors
 * on the "CAN2" high-speed bus. Geared in opposing pairs.
 */
class FRCFlywheelHardwareIO(
    private val leftMaster: TalonFX,
    private val leftFollower: TalonFX,
    private val rightMaster: TalonFX,
    private val rightFollower: TalonFX
) : FlywheelIO {

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
        leftMaster.optimizeBusUtilization()
        leftFollower.optimizeBusUtilization()
        rightMaster.optimizeBusUtilization()
        rightFollower.optimizeBusUtilization()

        setUpdateFrequencies(50.0, leftMasterVelocity, rightMasterVelocity)
        setUpdateFrequencies(20.0, leftMasterCurrent, leftFollowerCurrent, rightMasterCurrent, rightFollowerCurrent)
        setUpdateFrequencies(4.0, leftMasterTemp, rightMasterTemp)

        // Configure followers as opposed to their respective masters
        leftFollower.setControl(Follower(leftMaster.deviceID, com.ctre.phoenix6.signals.MotorAlignmentValue.Opposed))
        rightFollower.setControl(Follower(rightMaster.deviceID, com.ctre.phoenix6.signals.MotorAlignmentValue.Opposed))

        // Enforce exact physical configurations matching SystemConstants.java
        listOf(leftMaster, leftFollower, rightMaster, rightFollower).applyConfig {
            Slot0.kP = 0.5
            Slot0.kI = 0.0
            Slot0.kD = 0.0
            Slot0.kV = 0.12 // 12.0 / 100.0 (Max speed: 6000 RPM / 60 = 100 RPS)

            MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast
            MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive

            Feedback.SensorToMechanismRatio = 1.0

            Voltage.PeakReverseVoltage = 0.0 // Software lock reversal of flywheel
            CurrentLimits.SupplyCurrentLimitEnable = true
            CurrentLimits.SupplyCurrentLimit = 70.0
            CurrentLimits.StatorCurrentLimitEnable = true
            CurrentLimits.StatorCurrentLimit = 120.0
        }
    }



    override fun refresh() {
        BaseStatusSignal.refreshAll(
            leftMasterVelocity, rightMasterVelocity,
            leftMasterCurrent, leftFollowerCurrent, rightMasterCurrent, rightFollowerCurrent,
            leftMasterTemp, rightMasterTemp
        )
    }

    override fun setVelocityRpm(rpm: Double) {
        /**
         * Documentation for rps
         */
        val rps = rpm / 60.0
        leftMaster.setControl(velocityRequest.withVelocity(rps))
        rightMaster.setControl(velocityRequest.withVelocity(rps))
    }

    override fun setAppliedVoltage(volts: Double) {
        leftMaster.setControl(voltageRequest.withOutput(volts))
        rightMaster.setControl(voltageRequest.withOutput(volts))
    }

    override val velocityRpm: Double
        get() = (leftMasterVelocity.valueAsDouble + rightMasterVelocity.valueAsDouble) / 2.0 * 60.0

    override val currentAmps: Double
        get() = leftMasterCurrent.valueAsDouble +
                leftFollowerCurrent.valueAsDouble +
                rightMasterCurrent.valueAsDouble +
                rightFollowerCurrent.valueAsDouble

    override val tempCelsius: Double
        get() = Math.max(leftMasterTemp.valueAsDouble, rightMasterTemp.valueAsDouble)
}
