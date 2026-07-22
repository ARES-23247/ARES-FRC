package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.action.RobotAction

class MarvinFlywheelController(store: Store) : MarvinControllerBase(store) {
    private var lastFlywheelRpm = Double.NaN
    private var lastFlywheelActive: Boolean? = null
    /**
     * Documentation for flywheelRPM
     */

    val flywheelRPM: Double
        get() = store.state.superstructure.marvin.flywheel.velocityRpm
    /**
     * Documentation for flywheelTargetRPM
     */

    val flywheelTargetRPM: Double
        get() = store.state.superstructure.marvin.flywheel.targetVelocityRpm
    /**
     * Documentation for spinUp
     */

    fun spinUp(targetRpm: Double) {
        dispatchOnChange(lastFlywheelRpm, targetRpm, ::SetFlywheelSpeed) { lastFlywheelRpm = it }
        dispatchOnChange(lastFlywheelActive, true, ::SetFlywheelActive) { lastFlywheelActive = it }
    }
    /**
     * Documentation for stop
     */

    fun stop() {
        dispatchOnChange(lastFlywheelActive, false, ::SetFlywheelActive) { lastFlywheelActive = it }
    }
    /**
     * Documentation for isRpmAligned
     */

    fun isRpmAligned(targetRpm: Double): Boolean {
        return kotlin.math.abs(flywheelRPM - targetRpm) < 150.0
    }
}
