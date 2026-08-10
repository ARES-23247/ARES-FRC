package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.action.RobotAction

class MarvinFlywheelController(store: Store) : MarvinControllerBase(store) {
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
        dispatchOnChange(store.state.superstructure.marvin.flywheel.targetVelocityRpm, targetRpm, ::SetFlywheelSpeed) {}
        dispatchOnChange(store.state.superstructure.marvin.flywheelActive, true, ::SetFlywheelActive) {}
    }
    /**
     * Documentation for stop
     */

    fun stop() {
        dispatchOnChange(store.state.superstructure.marvin.flywheel.targetVelocityRpm, 0.0, ::SetFlywheelSpeed) {}
        dispatchOnChange(store.state.superstructure.marvin.flywheelActive, false, ::SetFlywheelActive) {}
    }
    /**
     * Documentation for isRpmAligned
     */

    fun isRpmAligned(targetRpm: Double): Boolean {
        val flywheel = store.state.superstructure.marvin.flywheel
        return flywheel.velocityValid && targetRpm > 100.0 && kotlin.math.abs(flywheelRPM - targetRpm) < 150.0
    }
}
