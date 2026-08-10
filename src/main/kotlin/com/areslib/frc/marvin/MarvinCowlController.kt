package com.areslib.frc.marvin

import com.areslib.Store

/** Redux facade for the cowl's mechanism-rotation target and software travel clamp. */
class MarvinCowlController(store: Store) : MarvinControllerBase(store) {

    /** Cached measured cowl position in mechanism rotations. */
    val cowlAngleRotations: Double
        get() = store.state.superstructure.marvin.cowl.angleRotations

    /** Commands mechanism rotations, clamped to the same limit configured in TalonFX IO. */
    fun setCowlAngleRotations(rotations: Double) {
        val clampedRotations = rotations.coerceIn(0.0, MarvinConfig.cowlMaxRotations)
        dispatchOnChange(store.state.superstructure.marvin.cowl.targetAngleRotations, clampedRotations, ::SetCowlAngle) {}
    }
}
