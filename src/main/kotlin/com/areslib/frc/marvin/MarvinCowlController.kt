package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.action.RobotAction

class MarvinCowlController(store: Store) : MarvinControllerBase(store) {
    private var lastCowlAngle = Double.NaN
    /**
     * Documentation for cowlAngleRotations
     */

    val cowlAngleRotations: Double
        get() = store.state.superstructure.marvin.cowl.angleRotations
    /**
     * Documentation for setCowlAngle
     */

    fun setCowlAngleRotations(rotations: Double) {
        val clampedRotations = rotations.coerceIn(0.0, MarvinConfig.cowlMaxRotations)
        dispatchOnChange(lastCowlAngle, clampedRotations, ::SetCowlAngle) { lastCowlAngle = it }
    }
}
