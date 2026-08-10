package com.areslib.frc.hardware

import com.areslib.frc.hardware.FeederIO
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX

/**
 * Concrete implementation of FeederIO utilizing a CTRE TalonFX motor on CAN2.
 * Note: Marvin 19 does not have a physical beam break sensor.
 */
class FRCFeederHardwareIO(
    private val motor: TalonFX
) : FeederIO {

    private val voltageRequest = VoltageOut(0.0)

    private val feederCurrent = motor.statorCurrent

    init {
        motor.optimizeBusUtilization()
        setUpdateFrequencies(10.0, feederCurrent)

        listOf(motor).applyConfig {
            MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast
            MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            Feedback.SensorToMechanismRatio = 4.0 // 4:1 feeder gear reduction

            CurrentLimits.SupplyCurrentLimitEnable = true
            CurrentLimits.SupplyCurrentLimit = 60.0
            CurrentLimits.StatorCurrentLimitEnable = true
            CurrentLimits.StatorCurrentLimit = 100.0
        }
    }

    override fun refresh() {
        feederCurrent.refresh()
    }

    /**
     * Drives the feeder open-loop via raw voltage. This is deliberate: the feeder
     * is a simple transfer roller with no closed-loop velocity requirement, so no
     * TalonFX Slot0 PID gains are configured. Voltage scaling is applied upstream
     * by [com.areslib.frc.marvin.MarvinSuperstructure] (FEEDER_KV * rps * brownoutScale).
     */
    override fun setAppliedVoltage(volts: Double) {
        motor.setControl(voltageRequest.withOutput(volts))
    }

    override val isBeamBroken: Boolean
        get() = false

    override val pieceDetectionValid: Boolean
        get() = false

    override val currentAmps: Double
        get() = feederCurrent.valueAsDouble
}
