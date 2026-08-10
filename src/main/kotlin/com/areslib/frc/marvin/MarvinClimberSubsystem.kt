package com.areslib.frc.marvin

import com.areslib.Store

class MarvinClimberSubsystem(store: Store) : MarvinControllerBase(store) {

    /**
     * Current climber mechanism position in motor rotations.
     */
    val positionRotations: Double
        get() = store.state.superstructure.marvin.climber.positionRotations

    /**
     * Commanded climber mechanism position in motor rotations.
     */
    val targetPositionRotations: Double
        get() = store.state.superstructure.marvin.climber.targetPositionRotations

    /**
     * Selects closed-loop position control with an explicit rotations target.
     */
    fun setTargetPositionRotations(rotations: Double) {
        val climber = store.state.superstructure.marvin.climber
        if (climber.controlMode != ClimberControlMode.POSITION_ROTATIONS || climber.targetPositionRotations != rotations) {
            store.dispatch(SetClimberPositionRotations(rotations))
        }
    }

    /**
     * Documentation for setVoltage
     */
    fun setVoltage(volts: Double) {
        val climber = store.state.superstructure.marvin.climber
        if (climber.controlMode != ClimberControlMode.VOLTAGE || climber.targetVoltage != volts) {
            store.dispatch(SetClimberVoltage(volts))
        }
    }
}
