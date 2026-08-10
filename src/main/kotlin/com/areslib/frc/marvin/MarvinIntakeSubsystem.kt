package com.areslib.frc.marvin

import com.areslib.Store

class MarvinIntakeSubsystem(store: Store) : MarvinControllerBase(store) {

    /**
     * Documentation for isDeployed
     */
    val isDeployed: Boolean
        get() = store.state.superstructure.marvin.intake.isDeployed

    /**
     * Documentation for pivotAngleDegrees
     */
    val pivotAngleDegrees: Double
        get() = store.state.superstructure.marvin.intake.pivotAngleDegrees

    /**
     * Documentation for rollerSpeedRps
     */
    val rollerSpeedRps: Double
        get() = store.state.superstructure.marvin.intake.rollerVelocityRps

    /**
     * Documentation for deploy
     */
    fun deploy() {
        dispatchOnChange(store.state.superstructure.marvin.intake.isDeployed, true, ::SetIntakePivot) {}
    }

    /**
     * Documentation for retract
     */
    fun retract() {
        dispatchOnChange(store.state.superstructure.marvin.intake.isDeployed, false, ::SetIntakePivot) {}
    }

    /**
     * Documentation for setRollerSpeed
     */
    fun setRollerSpeed(rps: Double) {
        dispatchOnChange(store.state.superstructure.marvin.intake.targetRollerVelocityRps, rps, ::SetIntakeRollers) {}
    }
}
