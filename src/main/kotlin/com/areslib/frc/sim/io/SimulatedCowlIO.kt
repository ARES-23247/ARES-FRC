package com.areslib.frc.sim.io

import com.areslib.frc.hardware.CowlIO
import com.areslib.frc.Dyn4jSimulation

class SimulatedCowlIO(private val sim: Dyn4jSimulation) : CowlIO {
    override fun setTargetAngle(degrees: Double) {
        val error = degrees - sim.simCowlAngle
        sim.simCowlVoltage = (error * 0.5).coerceIn(-12.0, 12.0)
    }
    override fun setAppliedVoltage(volts: Double) {
        sim.simCowlVoltage = volts.coerceIn(-12.0, 12.0)
    }
    override val angleDegrees: Double get() = sim.simCowlAngle
    override val currentAmps: Double get() = Math.abs(sim.simCowlVoltage) * 0.2
}
