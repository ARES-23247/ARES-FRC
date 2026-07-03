package com.areslib.frc.pathing

import com.areslib.control.HolonomicDriveController
import com.areslib.control.PIDController
import com.areslib.math.Pose2d
import com.areslib.math.Rotation2d
import com.areslib.frc.reducer.MarvinReducer
import com.areslib.state.RobotState
import com.areslib.state.DriveState
import com.areslib.state.SuperstructureMode
import com.areslib.action.RobotAction
import com.areslib.frc.action.*
import com.areslib.frc.state.marvinXIX
import com.areslib.pathing.PathPlannerParser
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertTrue
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

        // 2. Setup controller and state
        val driveController = HolonomicDriveController(
            PIDController(4.0, 0.0, 0.1),
            PIDController(4.0, 0.0, 0.1),
            PIDController(3.0, 0.0, 0.0)
        )

        val startPoint = path.points.first()
        var currentState = RobotState(
            drive = DriveState(
                odometryX = startPoint.pose.x,
                odometryY = startPoint.pose.y,
                odometryHeading = startPoint.pose.heading.radians
            )
        )

        var autoDistance = 0.0
        val dt = 0.02 // 20ms FRC loop
        val totalDistance = path.points.last().distanceMeters

        var flywheelOnTriggered = false
        var intakeDeployTriggered = false
        var feederShootTriggered = false

        // 3. Run E2E Trajectory simulation step-by-step
        while (autoDistance < totalDistance) {
            val currentPose = Pose2d(
                currentState.drive.odometryX,
                currentState.drive.odometryY,
                Rotation2d(currentState.drive.odometryHeading)
            )

            val targetPoint = path.sampleAtDistance(autoDistance)

            // Compute velocities
            val speeds = driveController.calculate(
                currentPose = currentPose,
                targetPose = targetPoint.pose,
                targetVelocityMps = targetPoint.velocityMps,
                targetHeading = targetPoint.pose.heading,
                dtSeconds = dt
            )

            // Update odometry to match target trajectory progress in this simulated step
            currentState = currentState.copy(
                drive = currentState.drive.copy(
                    odometryX = targetPoint.pose.x,
                    odometryY = targetPoint.pose.y,
                    odometryHeading = targetPoint.pose.heading.radians,
                    xVelocityMetersPerSecond = speeds.vxMetersPerSecond,
                    yVelocityMetersPerSecond = speeds.vyMetersPerSecond,
                    angularVelocityRadiansPerSecond = speeds.omegaRadiansPerSecond
                )
            )

            // Process event markers along the trajectory progress
            for (event in path.events) {
                val prevDistance = autoDistance
                val nextDistance = autoDistance + targetPoint.velocityMps * dt
                if (event.triggerDistanceMeters in prevDistance..nextDistance) {
                    val timeNow = System.currentTimeMillis()
                    when (event.eventName) {
                        "FlywheelOn" -> {
                            flywheelOnTriggered = true
                            currentState = MarvinReducer.reduce(currentState, RobotAction.SetFlywheelActive(true, timeNow))
                            currentState = MarvinReducer.reduce(currentState, SetFlywheelSpeed(4000.0, timeNow))
                        }
                        "IntakeDeploy" -> {
                            intakeDeployTriggered = true
                            currentState = MarvinReducer.reduce(currentState, RobotAction.SetIntakeActive(true, timeNow))
                            currentState = MarvinReducer.reduce(currentState, SetIntakePivot(true, timeNow))
                            currentState = MarvinReducer.reduce(currentState, SetIntakeRollers(15.0, timeNow))
                        }
                        "FeederShoot" -> {
                            feederShootTriggered = true
                            currentState = MarvinReducer.reduce(currentState, RobotAction.SetInventoryCount(1, timeNow))
                            currentState = MarvinReducer.reduce(currentState, SetFeederSpeed(20.0, timeNow))
                            currentState = MarvinReducer.reduce(currentState, RobotAction.SetTransferActive(true, timeNow))
                        }
                    }
                }
            }

            // Propagate distance along profile (coerced nominal velocity to prevent start/end 0Mps profile standstills)
            val nominalSpeed = if (targetPoint.velocityMps < 0.1) 1.5 else targetPoint.velocityMps
            autoDistance += nominalSpeed * dt
        }

        // 4. Validate all FRC Superstructure actions were correctly triggered and reduced
        assertTrue(flywheelOnTriggered, "FlywheelOn auto event should have triggered")
        assertTrue(intakeDeployTriggered, "IntakeDeploy auto event should have triggered")
        assertTrue(feederShootTriggered, "FeederShoot auto event should have triggered")

        // 5. Hard assertions on the final reduced state
        val superstructure = currentState.superstructure
        val marvin = superstructure.marvinXIX
        assertEquals(4000.0, marvin.flywheel.targetVelocityRpm, "Flywheel target RPM should be exactly 4000.0")
        assertTrue(marvin.intake.isDeployed, "Intake pivot should remain deployed")
        assertEquals(15.0, marvin.intake.targetRollerVelocityRps, "Intake roller target velocity should be 15.0 RPS")
        assertEquals(20.0, marvin.feeder.targetVelocityRps, "Feeder speed target should be 20.0 RPS")
    }
}
