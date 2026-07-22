package com.areslib.frc.marvin

import com.areslib.Store

class MarvinIntakeSubsystem(store: Store) : MarvinControllerBase(store) {
    private var lastDeployed: Boolean? = null
    private var lastRollerSpeed = Double.NaN

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
        dispatchOnChange(lastDeployed, true, ::SetIntakePivot) { lastDeployed = it }
    }

    /**
     * Documentation for retract
     */
    fun retract() {
        dispatchOnChange(lastDeployed, false, ::SetIntakePivot) { lastDeployed = it }
    }

    /**
     * Documentation for setRollerSpeed
     */
    fun setRollerSpeed(rps: Double) {
        dispatchOnChange(lastRollerSpeed, rps, ::SetIntakeRollers) { lastRollerSpeed = it }
    }
}
