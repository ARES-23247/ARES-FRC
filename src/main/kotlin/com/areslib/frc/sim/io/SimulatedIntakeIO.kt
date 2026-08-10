package com.areslib.frc.sim.io

import com.areslib.frc.hardware.IntakeIO
import com.areslib.frc.Dyn4jSimulation

/**
 * Simulation boundary for intake pivot degrees and roller voltage.
 *
 * Effort-scaled pivot commands cap voltage while retaining the requested angle, mirroring the
 * brownout contract of the hardware implementation.
 */
class SimulatedIntakeIO(private val sim: Dyn4jSimulation) : IntakeIO {
    override fun setPivotAngle(degrees: Double) {
        val error = degrees - sim.intakePivotSim.angleDegrees
        sim.simIntakePivotVoltage = (error * 0.4).coerceIn(-12.0, 12.0)
    }
    override fun setPivotAngle(degrees: Double, maxEffortScale: Double) {
        val error = degrees - sim.intakePivotSim.angleDegrees
        val maxVolts = 12.0 * maxEffortScale.coerceIn(0.0, 1.0)
        sim.simIntakePivotVoltage = (error * 0.4).coerceIn(-maxVolts, maxVolts)
    }
    override fun setPivotVoltage(volts: Double) {
        sim.simIntakePivotVoltage = volts.coerceIn(-12.0, 12.0)
    }
    override fun setRollerVoltage(volts: Double) {
        sim.simIntakeRollerVoltage = volts.coerceIn(-12.0, 12.0)
    }
    override val pivotAngleDegrees: Double get() = sim.intakePivotSim.angleDegrees
    override val pivotCurrentAmps: Double get() = Math.abs(sim.simIntakePivotVoltage) * 0.3
    override val rollerCurrentAmps: Double get() = Math.abs(sim.simIntakeRollerVoltage) * 0.2
}
