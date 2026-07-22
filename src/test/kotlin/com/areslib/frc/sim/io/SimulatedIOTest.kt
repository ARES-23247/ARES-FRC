package com.areslib.frc.sim.io

import com.areslib.frc.Dyn4jSimulation
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
/**
 * Documentation for SimulatedIOTest
 */

class SimulatedIOTest {
    /**
     * Documentation for testAllSimulatedIOs
     */

    @Test
    fun testAllSimulatedIOs() {
        /**
         * Documentation for sim
         */
        val sim = Dyn4jSimulation(seed = 42L)

        // 1. Cowl
        /**
         * Documentation for cowl
         */
        val cowl = SimulatedCowlIO(sim)
        cowl.setTargetAngle(10.0)
        assertTrue(sim.simCowlVoltage != 0.0)
        cowl.setAppliedVoltage(5.0)
        assertEquals(5.0, sim.simCowlVoltage)
        assertEquals(sim.simCowlAngle, cowl.angleRotations)
        assertEquals(1.0, cowl.currentAmps)

        // 2. Climber
        /**
         * Documentation for climber
         */
        val climber = SimulatedClimberIO(sim)
        climber.setTargetExtension(0.1)
        assertTrue(sim.simClimberVoltage != 0.0)
        climber.setAppliedVoltage(-6.0)
        assertEquals(-6.0, sim.simClimberVoltage)
        assertEquals(sim.simClimberExtensionMeters, climber.extensionMeters)
        assertEquals(1.5, climber.currentAmps)

        // 3. Intake
        /**
         * Documentation for intake
         */
        val intake = SimulatedIntakeIO(sim)
        intake.setPivotAngle(45.0)
        assertTrue(sim.simIntakePivotVoltage != 0.0)
        intake.setPivotVoltage(8.0)
        assertEquals(8.0, sim.simIntakePivotVoltage)
        intake.setRollerVoltage(10.0)
        assertEquals(10.0, sim.simIntakeRollerVoltage)
        assertEquals(sim.intakePivotSim.angleDegrees, intake.pivotAngleDegrees)
        assertEquals(2.4, intake.pivotCurrentAmps, 1e-6)
        assertEquals(2.0, intake.rollerCurrentAmps, 1e-6)

        // 4. Feeder
        /**
         * Documentation for feeder
         */
        val feeder = SimulatedFeederIO(sim)
        feeder.setAppliedVoltage(4.0)
        assertEquals(4.0, sim.simFeederVoltage)
        assertFalse(feeder.isBeamBroken)
        assertEquals(0.4, feeder.currentAmps, 1e-6)

        // 5. Floor
        /**
         * Documentation for floor
         */
        val floor = SimulatedFloorIO(sim)
        floor.setAppliedVoltage(3.0)
        assertEquals(3.0, sim.simFloorVoltage)
        assertEquals(0.45, floor.currentAmps, 1e-6)
        assertEquals(sim.simFloorVelocityRps, floor.velocityRps)

        // 6. Flywheel
        /**
         * Documentation for flywheel
         */
        val flywheel = SimulatedFlywheelIO(sim)
        flywheel.setVelocityRpm(4000.0)
        assertTrue(sim.simFlywheelVoltage != 0.0)
        flywheel.setAppliedVoltage(9.0)
        assertEquals(9.0, sim.simFlywheelVoltage)
        assertEquals(sim.flywheelSim.velocityRpm, flywheel.velocityRpm)
        assertTrue(flywheel.currentAmps >= 0.0)
        assertEquals(30.0, flywheel.tempCelsius)
    }
}
