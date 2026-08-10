package com.areslib.frc.marvin

import com.areslib.Store

/** Redux facade for RPM commands and the fail-closed flywheel readiness gate. */
class MarvinFlywheelController(store: Store) : MarvinControllerBase(store) {

    /** Cached measured flywheel speed in RPM. Check freshness before safety decisions. */
    val flywheelRPM: Double
        get() = store.state.superstructure.marvin.flywheel.velocityRpm

    /** Current commanded flywheel speed in RPM. */
    val flywheelTargetRPM: Double
        get() = store.state.superstructure.marvin.flywheel.targetVelocityRpm

    /** Enables flywheel output and records [targetRpm] in RPM. */
    fun spinUp(targetRpm: Double) {
        dispatchOnChange(store.state.superstructure.marvin.flywheel.targetVelocityRpm, targetRpm, ::SetFlywheelSpeed) {}
        dispatchOnChange(store.state.superstructure.marvin.flywheelActive, true, ::SetFlywheelActive) {}
    }

    /** Disables flywheel output and clears the RPM target for safe re-commanding. */
    fun stop() {
        dispatchOnChange(store.state.superstructure.marvin.flywheel.targetVelocityRpm, 0.0, ::SetFlywheelSpeed) {}
        dispatchOnChange(store.state.superstructure.marvin.flywheelActive, false, ::SetFlywheelActive) {}
    }

    /** True only for a fresh sample within 150 RPM of a nontrivial target. */
    fun isRpmAligned(targetRpm: Double): Boolean {
        val flywheel = store.state.superstructure.marvin.flywheel
        return flywheel.velocityValid && targetRpm > 100.0 && kotlin.math.abs(flywheelRPM - targetRpm) < 150.0
    }
}
