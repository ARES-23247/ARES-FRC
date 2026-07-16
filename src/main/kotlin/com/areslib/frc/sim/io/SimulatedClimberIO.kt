package com.areslib.frc.sim.io

import com.areslib.frc.hardware.ClimberIO
import com.areslib.frc.Dyn4jSimulation

class SimulatedClimberIO(private val sim: Dyn4jSimulation) : ClimberIO {
    override fun setTargetExtension(meters: Double) {
        val error = meters - sim.simClimberExtensionMeters
        sim.simClimberVoltage = (error * 10.0).coerceIn(-12.0, 12.0)
    }
    override fun setAppliedVoltage(volts: Double) {
        sim.simClimberVoltage = volts.coerceIn(-12.0, 12.0)
    }
    override val extensionMeters: Double get() = sim.simClimberExtensionMeters
    override val currentAmps: Double get() = Math.abs(sim.simClimberVoltage) * 0.25
}
