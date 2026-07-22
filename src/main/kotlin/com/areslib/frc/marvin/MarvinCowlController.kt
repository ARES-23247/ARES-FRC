package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.action.RobotAction

class MarvinCowlController(private val store: Store) {
    private var lastCowlAngle = Double.NaN

    private inline fun <T> dispatchOnChange(
        current: T?,
        target: T,
        actionFactory: (T, Long) -> RobotAction,
        updateCurrent: (T) -> Unit
    ) {
        if (current != target) {
            store.dispatch(actionFactory(target, com.areslib.util.RobotClock.currentTimeMillis()))
            updateCurrent(target)
        }
    }

    val cowlAngleRotations: Double
        get() = store.state.superstructure.marvin.cowl.angleRotations

    fun setCowlAngle(degrees: Double) {
        dispatchOnChange(lastCowlAngle, degrees, ::SetCowlAngle) { lastCowlAngle = it }
    }
}
