package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.action.RobotAction

class MarvinFlywheelController(private val store: Store) {
    private var lastFlywheelRpm = Double.NaN
    private var lastFlywheelActive: Boolean? = null

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

    val flywheelRPM: Double
        get() = store.state.superstructure.marvin.flywheel.velocityRpm

    val flywheelTargetRPM: Double
        get() = store.state.superstructure.marvin.flywheel.targetVelocityRpm

    fun spinUp(targetRpm: Double) {
        dispatchOnChange(lastFlywheelRpm, targetRpm, ::SetFlywheelSpeed) { lastFlywheelRpm = it }
        dispatchOnChange(lastFlywheelActive, true, ::SetFlywheelActive) { lastFlywheelActive = it }
    }

    fun stop() {
        dispatchOnChange(lastFlywheelActive, false, ::SetFlywheelActive) { lastFlywheelActive = it }
    }

    fun isRpmAligned(targetRpm: Double): Boolean {
        return kotlin.math.abs(flywheelRPM - targetRpm) < 150.0
    }
}
