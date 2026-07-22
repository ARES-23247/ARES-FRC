package com.areslib.frc.sim.io

import com.areslib.frc.hardware.IntakeIO
import com.areslib.frc.Dyn4jSimulation

class SimulatedIntakeIO(private val sim: Dyn4jSimulation) : IntakeIO {
    override fun setPivotAngle(degrees: Double) {
        /**
         * Documentation for error
         */
        val error = degrees - sim.intakePivotSim.angleDegrees
        sim.simIntakePivotVoltage = (error * 0.4).coerceIn(-12.0, 12.0)
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
