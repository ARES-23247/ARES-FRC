package com.areslib.frc

import com.areslib.action.RobotAction
import com.areslib.state.*
import com.areslib.telemetry.ITelemetry
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import org.dyn4j.dynamics.Body

class Dyn4jSimulationTest {

    @Suppress("UNCHECKED_CAST")
    @Test
    fun testHighCapacityInventoryLimit() {
        val sim = Dyn4jSimulation(seed = 42L)
        val state = RobotState(superstructure = SuperstructureState(inventoryCount = 39))

        // Get private 'balls' field via reflection
        val ballsField = Dyn4jSimulation::class.java.getDeclaredField("balls")
        ballsField.isAccessible = true
        val ballsList = ballsField.get(sim) as MutableList<Body>

        // Assert that we have spawned balls
        assertTrue(ballsList.isNotEmpty())

        // Deploy intake and start spinning rollers in simulation
        sim.intakeIO.setPivotAngle(90.0)
        sim.intakeIO.setRollerVoltage(12.0)

        // Force one of the balls to be exactly at the robot's center (2.0, 2.0)
        ballsList[0].transform.setTranslation(2.0, 2.0)

        // Run step with intake deployed and active.
        // First run a few steps to let the pivot simulator update past 45 degrees
        var pivotDegrees = 0.0
        val allActions = mutableListOf<RobotAction>()
        for (i in 0..50) {
            val stepActions = sim.step(state, 0.02)
            allActions.addAll(stepActions)
            pivotDegrees = sim.intakeIO.pivotAngleDegrees
            if (pivotDegrees > 45.0 && stepActions.any { it is RobotAction.SetInventoryCount }) break
        }
        assertTrue(pivotDegrees > 45.0, "Intake pivot should have deployed beyond 45 degrees")

        // Verify that ingestion was successful and we got SetInventoryCount action for 40
        val inventoryAction = allActions.find { it is RobotAction.SetInventoryCount } as? RobotAction.SetInventoryCount
        assertNotNull(inventoryAction, "Should have triggered ball ingestion action")
        assertEquals(40, inventoryAction!!.count, "Ingestion should successfully reach 40 balls")
    }

    @Suppress("UNCHECKED_CAST")
    @Test
    fun testShootingAnd2_5DProjectileMotion() {
        val sim = Dyn4jSimulation(seed = 42L)
        
        // Setup state with active flywheel ready at 4000 RPM, and cowl angle
        val superstructure = SuperstructureState(
            flywheelActive = true,
            flywheelRPM = 4000.0,
            inventoryCount = 10
        )
        val state = RobotState(superstructure = superstructure)

        // Set flywheel RPM instantly using reflection
        val flywheelSimField = Dyn4jSimulation::class.java.getDeclaredField("flywheelSim")
        flywheelSimField.isAccessible = true
        val flywheelSimInstance = flywheelSimField.get(sim) as com.areslib.sim.FlywheelSim
        val angularVelField = com.areslib.sim.FlywheelSim::class.java.getDeclaredField("angularVelocityRadPerSec")
        angularVelField.isAccessible = true
        angularVelField.set(flywheelSimInstance, 4000.0 * 2.0 * Math.PI / 60.0)

        // Set cowl angle instantly to 30.0 degrees using reflection
        val cowlAngleField = Dyn4jSimulation::class.java.getDeclaredField("simCowlAngle")
        cowlAngleField.isAccessible = true
        cowlAngleField.set(sim, 30.0)

        // Set feeder voltage to trigger ingestion/shoot
        sim.feederIO.setAppliedVoltage(12.0)

        // Run step forward to let cowl angle update and trigger shoot
        var actions: List<RobotAction> = emptyList()
        for (i in 0..20) {
            actions = sim.step(state, 0.02)
            if (actions.any { it is RobotAction.SetInventoryCount }) break
        }

        // Verify shoot action dispatched decrement
        val shootAction = actions.find { it is RobotAction.SetInventoryCount } as? RobotAction.SetInventoryCount
        assertNotNull(shootAction, "Should have triggered a shooting action")
        assertEquals(9, shootAction!!.count)

        // Check private 'flyingBalls' list via reflection
        val flyingField = Dyn4jSimulation::class.java.getDeclaredField("flyingBalls")
        flyingField.isAccessible = true
        val flyingList = flyingField.get(sim) as List<FlyingBall>

        assertEquals(1, flyingList.size, "Exactly one ball should be flying in 2.5D space")
        val fb = flyingList[0]
        assertEquals(0.7164096, fb.z, 1e-4, "Initial launch height should be exactly 0.7164096 meters after one step")
        assertTrue(fb.vx > 0.0 || fb.vy > 0.0, "Horizontal velocities should be non-zero")
        assertTrue(fb.vz > 0.0, "Vertical velocity should be positive due to cowl angle")
    }

    @Suppress("UNCHECKED_CAST")
    @Test
    fun testHubScoringAndCenterEjection() {
        val sim = Dyn4jSimulation(seed = 42L)
        val state = RobotState()

        // Get private 'flyingBalls' list via reflection
        val flyingField = Dyn4jSimulation::class.java.getDeclaredField("flyingBalls")
        flyingField.isAccessible = true
        val flyingList = flyingField.get(sim) as MutableList<FlyingBall>

        // Get private 'balls' field via reflection to count ground balls before scoring
        val ballsField = Dyn4jSimulation::class.java.getDeclaredField("balls")
        ballsField.isAccessible = true
        val ballsList = ballsField.get(sim) as MutableList<Body>
        val initialGroundBalls = ballsList.size

        // Mock a flying ball inside the Blue Hub cylindrical scoring zone (center: 4.135, 4.0345)
        // at height z = 2.0 (inside 1.6 to 2.8 meters range)
        val scoredBall = FlyingBall(
            x = 4.135,
            y = 4.0345,
            z = 2.0,
            vx = 0.1,
            vy = 0.1,
            vz = -1.0
        )
        flyingList.add(scoredBall)

        // Step simulation
        sim.step(state, 0.02)

        // Verify flying ball was removed
        assertTrue(flyingList.isEmpty(), "Scored ball should be removed from flyingBalls list")

        // Verify new ground ball was spawned (balls size increased)
        assertEquals(initialGroundBalls + 1, ballsList.size, "A new ground ball should be spawned")

        // Verify the newest ball is spawned at the center of the field (8.2705, 4.0345)
        val newestBall = ballsList.last()
        assertEquals(8.2705, newestBall.transform.translationX, 0.05, "Scored ball should eject to field X center")
        assertEquals(4.0345, newestBall.transform.translationY, 0.05, "Scored ball should eject to field Y center")
        
        // Verify it has non-zero ejection velocities
        assertTrue(newestBall.linearVelocity.magnitude > 1.0, "Ejected ball should have an outward velocity")
    }

    @Suppress("UNCHECKED_CAST")
    @Test
    fun testLandingOnGround() {
        val sim = Dyn4jSimulation(seed = 42L)
        val state = RobotState()

        // Get private 'flyingBalls' list via reflection
        val flyingField = Dyn4jSimulation::class.java.getDeclaredField("flyingBalls")
        flyingField.isAccessible = true
        val flyingList = flyingField.get(sim) as MutableList<FlyingBall>

        // Mock a ball almost landing on the ground (z = 0.05, radius is 0.0635)
        val landingBall = FlyingBall(
            x = 6.0,
            y = 5.0,
            z = 0.05,
            vx = 3.0,
            vy = -2.0,
            vz = -5.0
        )
        flyingList.add(landingBall)

        // Step simulation
        sim.step(state, 0.02)

        // Verify flying ball was removed
        assertTrue(flyingList.isEmpty(), "Landed ball should be removed from flying list")

        // Get balls list to verify landing body translation and velocity
        val ballsField = Dyn4jSimulation::class.java.getDeclaredField("balls")
        ballsField.isAccessible = true
        val ballsList = ballsField.get(sim) as List<Body>

        val dynamicBody = ballsList.last()
        assertEquals(6.0, dynamicBody.transform.translationX, 0.1, "Landed ball should match final X position")
        assertEquals(5.0, dynamicBody.transform.translationY, 0.1, "Landed ball should match final Y position")
        assertEquals(3.0, dynamicBody.linearVelocity.x, 1e-3, "Residual linear velocity X should be preserved")
        assertEquals(-2.0, dynamicBody.linearVelocity.y, 1e-3, "Residual linear velocity Y should be preserved")
    }

    @Suppress("UNCHECKED_CAST")
    @Test
    fun testTelemetryPackaging() {
        val sim = Dyn4jSimulation(seed = 42L)
        val state = RobotState()

        // Get private lists via reflection
        val flyingField = Dyn4jSimulation::class.java.getDeclaredField("flyingBalls")
        flyingField.isAccessible = true
        val flyingList = flyingField.get(sim) as MutableList<FlyingBall>

        val ballsField = Dyn4jSimulation::class.java.getDeclaredField("balls")
        ballsField.isAccessible = true
        val ballsList = ballsField.get(sim) as List<Body>

        // Mock a flying ball
        flyingList.add(FlyingBall(3.0, 3.0, 2.5, 0.0, 0.0, 0.0))

        val mockTelemetry = object : ITelemetry {
            val arrays = mutableMapOf<String, DoubleArray>()
            override fun putDoubleArray(key: String, value: DoubleArray) {
                arrays[key] = value
            }
            override fun putNumber(key: String, value: Double) {}
            override fun putString(key: String, value: String) {}
            override fun putBoolean(key: String, value: Boolean) {}
            override fun getNumber(key: String, defaultValue: Double): Double = defaultValue
            override fun getBoolean(key: String, defaultValue: Boolean): Boolean = defaultValue
            override fun getString(key: String, defaultValue: String): String = defaultValue
        }

        sim.publishVisualization(state, mockTelemetry)

        // Verify the double array exists and has correct sizing
        val poseArray = mockTelemetry.arrays["Robot/FuelPoses"]
        assertNotNull(poseArray)
        assertEquals((ballsList.size + 1) * 7, poseArray!!.size)

        // Verify the flying ball is packaged at the correct index at the end
        val lastIdx = ballsList.size * 7
        assertEquals(3.0, poseArray[lastIdx])
        assertEquals(3.0, poseArray[lastIdx + 1])
        assertEquals(2.5, poseArray[lastIdx + 2], "Flying ball Z coordinate must be published correctly")
        assertEquals(1.0, poseArray[lastIdx + 3], "Identity quaternion qw must be 1.0")
    }

    @Test
    fun testCowlAngleUnitMapping() {
        val sim = Dyn4jSimulation()
        
        // Step simulation forward for 2 seconds (100 steps of 0.02) to let closed-loop settle
        val state = RobotState()
        for (i in 0 until 100) {
            sim.cowlIO.setTargetAngle(1.0)
            sim.step(state, 0.02)
        }
        
        // The simulated cowl angle should settle near 32.0 degrees (1.0 rotations * 32.0)
        // And angleDegrees (feedback) should return 1.0 mechanism rotations.
        assertEquals(1.0, sim.cowlIO.angleDegrees, 0.05, "Cowl feedback should match commanded rotations")
    }
}
