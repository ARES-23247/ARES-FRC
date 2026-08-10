package com.areslib.frc.pathing

import com.areslib.frc.Dyn4jSimulation
import com.areslib.frc.FrcSwerveRobot
import com.areslib.frc.marvin.MarvinConfig
import com.areslib.frc.marvin.MarvinIntakeSubsystem
import com.areslib.frc.marvin.MarvinReducer
import com.areslib.frc.marvin.MarvinShooterSubsystem
import com.areslib.frc.marvin.MarvinState
import com.areslib.frc.marvin.SetFlywheelSpeed
import com.areslib.frc.marvin.SuperstructureSensorUpdate
import com.areslib.frc.marvin.marvin
import com.areslib.frc.robot.FRCAutoOrchestrator
import com.areslib.pathing.PathPlannerParser
import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import java.io.BufferedReader
import java.io.InputStreamReader

class E2EAutonomousSimulationTest {

    @Test
    fun testE2EAutonomousTrajectoryAndSubsystems() {
        // 1. Load SimPath.path from test classpath resources (inherited from main resources)
        val resourcePath = "/deploy/pathplanner/paths/SimPath.path"
        val inputStream = javaClass.getResourceAsStream(resourcePath)
        assertNotNull(inputStream, "Could not find SimPath.path resource in test classpath!")

        val jsonString = BufferedReader(InputStreamReader(inputStream!!, Charsets.UTF_8)).use { reader ->
            reader.readText()
        }
        val path = PathPlannerParser.parsePath(jsonString)
        assertNotNull(path)
        assertTrue(path.points.isNotEmpty(), "Parsed path points should not be empty")
        assertEquals(3, path.events.size, "Should have exactly 3 parsed event markers (FlywheelOn, IntakeDeploy, FeederShoot)")

        // 2. Build production FrcSwerveRobot + FRCAutoOrchestrator instances so events flow through the
        //    production dispatch path rather than a reimplementation in the test.
        val robot = FrcSwerveRobot(
            isSimulation = true,
            initialState = RobotState(
                superstructure = SuperstructureState(custom = MarvinState())
            ),
            reducer = { state, action -> MarvinReducer.reduce(state, action) }
        )
        val sim = Dyn4jSimulation()
        val marvinShooter = MarvinShooterSubsystem(robot.store)
        val marvinIntake = MarvinIntakeSubsystem(robot.store)
        val orchestrator = FRCAutoOrchestrator(robot, sim, marvinShooter, marvinIntake)

        // 3. Seed an aligned flywheel (target + measured velocity) so the FeederShoot event
        //    fires immediately instead of waiting on RPM convergence.
        robot.store.dispatch(SetFlywheelSpeed(4000.0, 1000L))
        robot.store.dispatch(
            SuperstructureSensorUpdate(
                flywheelRpm = 4000.0,
                cowlAngleRotations = 0.0,
                intakeAngle = 0.0,
                pieceDetected = false,
                flywheelVelocityValid = true,
                timestampMs = 1000L
            )
        )

        // 4. Drive each parsed event through the production orchestrator handler in declared order.
        var flywheelOnCompleted = false
        var intakeDeployCompleted = false
        var feederShootCompleted = false
        var t = 1.0
        for (event in path.events) {
            if (orchestrator.handleEvent(event.eventName, t)) {
                when (event.eventName) {
                    "FlywheelOn" -> flywheelOnCompleted = true
                    "IntakeDeploy" -> intakeDeployCompleted = true
                    "FeederShoot" -> feederShootCompleted = true
                }
            }
            t += 0.02
        }

        // 5. Every event must have completed through the production code path.
        assertTrue(flywheelOnCompleted, "FlywheelOn auto event should have completed via the real orchestrator")
        assertTrue(intakeDeployCompleted, "IntakeDeploy auto event should have completed via the real orchestrator")
        assertTrue(feederShootCompleted, "FeederShoot auto event should have completed via the real orchestrator")

        // 6. Hard assertions on the resulting Redux state.
        val marvin = robot.store.state.superstructure.marvin
        assertEquals(4000.0, marvin.flywheel.targetVelocityRpm, "FlywheelOn should spin the flywheel up to 4000 RPM")
        assertTrue(marvin.intake.isDeployed, "IntakeDeploy should deploy the intake pivot")
        assertEquals(15.0, marvin.intake.targetRollerVelocityRps, "IntakeDeploy should set roller target to 15.0 RPS")
        assertTrue(marvin.transferActive, "FeederShoot should enable transfer")
        assertNotEquals(0.0, marvin.feeder.targetVelocityRps, "FeederShoot MUST dispatch a non-zero feeder speed (regression: previously 0V)")
        assertEquals(MarvinConfig.FEEDER_SHOOT_SPEED_RPS, marvin.feeder.targetVelocityRps, "Feeder speed target should match the configured shoot speed")
    }
}
