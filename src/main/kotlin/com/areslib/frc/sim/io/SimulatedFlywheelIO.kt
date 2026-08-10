package com.areslib.frc.sim.io

import com.areslib.frc.hardware.FlywheelIO
import com.areslib.frc.Dyn4jSimulation

/**
 * Simulation boundary for flywheel commands in RPM and voltage.
 *
 * Velocity is always valid because it is read from the in-process model rather than transported
 * across CAN; voltage commands are clamped to the nominal 12 V bus.
 */
class SimulatedFlywheelIO(private val sim: Dyn4jSimulation) : FlywheelIO {
    override fun setVelocityRpm(rpm: Double) {
        val error = rpm - sim.flywheelSim.velocityRpm
        sim.simFlywheelVoltage = (error * 0.003).coerceIn(-12.0, 12.0)
    }
    override fun setAppliedVoltage(volts: Double) {
        sim.simFlywheelVoltage = volts.coerceIn(-12.0, 12.0)
    }
    override val velocityRpm: Double get() = sim.flywheelSim.velocityRpm
    override val velocityValid: Boolean get() = true
    override val currentAmps: Double get() = sim.flywheelSim.getCurrentAmps(sim.simFlywheelVoltage)
    override val tempCelsius: Double get() = 30.0
}
