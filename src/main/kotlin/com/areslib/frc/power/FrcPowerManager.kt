package com.areslib.frc.power

import com.areslib.control.BrownoutGuard

/**
 * Manages battery voltage sag filtering and brownout protection for FRC superstructure motors.
 */
class FrcPowerManager {
    /** FRC brownout protection guard */
    val brownoutGuard = BrownoutGuard.frcDefaults()

    /** Supplier for the battery voltage, configurable by the robot platform */
    var batteryVoltageSupplier: () -> Double = { 12.6 }

    var batteryVoltage = 12.6
        private set
    var powerScale = 1.0
        private set

    /**
     * Updates the battery voltage and brownout guard status.
     * @return The calculated power scale factor (0.0 to 1.0).
     */
    fun update(): Double {
        batteryVoltage = batteryVoltageSupplier()
        brownoutGuard.update(batteryVoltage)
        powerScale = brownoutGuard.powerScale

        // Dynamically distribute powerScale to all registered motors
        val motors = com.areslib.hardware.HardwareRegistry.getRegisteredMotors()
        for (m in motors) {
            m.powerScale = powerScale
        }

        return powerScale
    }
}
