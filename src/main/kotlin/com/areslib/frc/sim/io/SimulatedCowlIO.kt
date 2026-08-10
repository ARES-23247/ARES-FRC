package com.areslib.frc.sim.io

import com.areslib.frc.hardware.CowlIO
import com.areslib.frc.Dyn4jSimulation

/**
 * Simulation boundary for cowl mechanism rotations.
 *
 * The visualization model stores degrees internally and uses 32 degrees per mechanism rotation;
 * callers remain insulated from that representation through [CowlIO]. Effort-scaled position
 * commands cap voltage without changing the requested geometry.
 */
class SimulatedCowlIO(private val sim: Dyn4jSimulation) : CowlIO {
    override fun setTargetAngle(rotations: Double) {
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
