package com.areslib.frc.marvin

import com.areslib.Store

/** Redux facade for the intake's two pivot positions and roller RPS target. */
class MarvinIntakeSubsystem(store: Store) : MarvinControllerBase(store) {

    /** Commanded logical pivot state, not a measured limit-switch state. */
    val isDeployed: Boolean
        get() = store.state.superstructure.marvin.intake.isDeployed

    /** Cached measured pivot angle in degrees. */
    val pivotAngleDegrees: Double
        get() = store.state.superstructure.marvin.intake.pivotAngleDegrees

    /** Cached measured roller speed in RPS when supplied by IO. */
    val rollerSpeedRps: Double
        get() = store.state.superstructure.marvin.intake.rollerVelocityRps

    /** Commands the calibrated 90-degree deployed position. */
    fun deploy() {
        dispatchOnChange(store.state.superstructure.marvin.intake.isDeployed, true, ::SetIntakePivot) {}
    }

    /** Commands the calibrated zero-degree stowed position. */
    fun retract() {
        dispatchOnChange(store.state.superstructure.marvin.intake.isDeployed, false, ::SetIntakePivot) {}
    }

    /** Commands intake roller speed in revolutions per second. */
    fun setRollerSpeed(rps: Double) {
        dispatchOnChange(store.state.superstructure.marvin.intake.targetRollerVelocityRps, rps, ::SetIntakeRollers) {}
    }
}
