package com.areslib.frc.sim.io

import com.areslib.frc.hardware.CowlIO
import com.areslib.frc.Dyn4jSimulation

class SimulatedCowlIO(private val sim: Dyn4jSimulation) : CowlIO {
    override fun setTargetAngle(rotations: Double) {
        /**
         * Documentation for error
         */
        val targetDegrees = rotations * 32.0
        val error = targetDegrees - sim.simCowlAngle
        sim.simCowlVoltage = (error * 0.5).coerceIn(-12.0, 12.0)
    }
    override fun setTargetAngle(rotations: Double, maxEffortScale: Double) {
        val targetDegrees = rotations * 32.0
        val error = targetDegrees - sim.simCowlAngle
        val maxVolts = 12.0 * maxEffortScale.coerceIn(0.0, 1.0)
        sim.simCowlVoltage = (error * 0.5).coerceIn(-maxVolts, maxVolts)
    }
    override fun setAppliedVoltage(volts: Double) {
        sim.simCowlVoltage = volts.coerceIn(-12.0, 12.0)
    }
    override val angleRotations: Double get() = sim.simCowlAngle / 32.0
    override val currentAmps: Double get() = Math.abs(sim.simCowlVoltage) * 0.2
}
