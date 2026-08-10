package com.areslib.frc.marvin

import com.areslib.Store

/** Redux facade for mutually exclusive climber voltage and position control modes. */
class MarvinClimberSubsystem(store: Store) : MarvinControllerBase(store) {

    /**
     * Current climber position in mechanism rotations.
     */
    val positionRotations: Double
        get() = store.state.superstructure.marvin.climber.positionRotations

    /**
     * Commanded climber position in mechanism rotations.
     */
    val targetPositionRotations: Double
        get() = store.state.superstructure.marvin.climber.targetPositionRotations

    /**
     * Selects closed-loop position control with an explicit mechanism-rotation target.
     */
    fun setTargetPositionRotations(rotations: Double) {
        val climber = store.state.superstructure.marvin.climber
        if (climber.controlMode != ClimberControlMode.POSITION_ROTATIONS || climber.targetPositionRotations != rotations) {
            store.dispatch(SetClimberPositionRotations(rotations))
        }
    }

    /** Selects open-loop voltage mode. */
    fun setVoltage(volts: Double) {
        val climber = store.state.superstructure.marvin.climber
        if (climber.controlMode != ClimberControlMode.VOLTAGE || climber.targetVoltage != volts) {
            store.dispatch(SetClimberVoltage(volts))
        }
    }
}
