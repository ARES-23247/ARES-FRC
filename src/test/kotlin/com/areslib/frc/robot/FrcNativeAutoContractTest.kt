package com.areslib.frc.robot

import com.areslib.action.RobotAction
import com.areslib.frc.FrcSwerveRobot
import com.areslib.frc.generated.GeneratedAresProject
import com.areslib.frc.marvin.MarvinReducer
import com.areslib.frc.marvin.MarvinState
import com.areslib.frc.marvin.SetFlywheelSpeed
import com.areslib.frc.marvin.SuperstructureSensorUpdate
import com.areslib.frc.marvin.marvin
import com.areslib.math.coordinate.CoordinateTransformers
import com.areslib.pathing.NamedCommands
import com.areslib.state.Alliance
import com.areslib.state.RobotState
import com.areslib.state.RoutineExecutionStatus
import com.areslib.state.SuperstructureState
import com.areslib.util.RobotClock
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

/** End-to-end contract for generated assets, capability factories, and FRC native execution. */
class FrcNativeAutoContractTest {
    @BeforeEach
    fun setUp() {
        RobotClock.useMockTime(1_000L)
        NamedCommands.clear()
        FrcAutoCapabilities.register()
    }

    @AfterEach
    fun tearDown() {
        NamedCommands.clear()
        RobotClock.useSystemTime()
    }

    @Test
    fun `generated action keys and source runtime registry are identical`() {
        val declared = FrcAutoCapabilities.descriptors.map { it.key.value }.toSet()
        val registered = NamedCommands.catalog().map { it.key.value }.toSet()

        assertEquals(declared, GeneratedAresProject.knownActionKeys)
        assertEquals(declared, registered)
        assertEquals(setOf("shooter.ready"), GeneratedAresProject.knownConditionKeys)
    }

    @Test
    fun `every generated autonomous entry preflights for both alliances`() {
        assertTrue(GeneratedAresProject.autonomousEntries.isNotEmpty())
        GeneratedAresProject.autonomousEntries.filter { it.enabled }.forEach { entry ->
            Alliance.entries.forEach { alliance ->
                val runner = runner(newRobot(alliance)) { entry.entryId }
                runner.autonomousInit()
                assertFalse(
                    runner.isFaultedForTest,
                    "${entry.entryId} failed $alliance preflight: ${runner.statusForTest}"
                )
            }
        }
    }

    @Test
    fun `red generated start pose reflects across the FRC alliance wall`() {
        val robot = newRobot(Alliance.RED)
        val runner = runner(robot) { "sim-drive-and-shoot" }

        runner.autonomousInit()

        val pose = robot.store.state.drive.poseEstimator.estimatedPose
        assertEquals(CoordinateTransformers.FRC_FIELD_LENGTH - 2.0, pose.x, 1e-9)
        assertEquals(2.0, pose.y, 1e-9)
        assertEquals(-Math.PI, pose.heading.radians, 1e-9)
        assertFalse(runner.isFaultedForTest)
    }

    @Test
    fun `selection is locked at autonomous init`() {
        var requested = "sim-drive-and-shoot"
        val runner = runner(newRobot(Alliance.BLUE)) { requested }

        runner.autonomousInit()
        requested = "do-nothing"
        runner.autonomousPeriodic()

        assertEquals("sim-drive-and-shoot", runner.selectedAutoForTest)
    }

    @Test
    fun `missing selection runs generated do nothing fallback and completes safely`() {
        val robot = newRobot(Alliance.BLUE)
        robot.drive.joystickDrive(2.0, -1.0, 0.5, isFieldCentric = false)
        val runner = runner(robot) { "deleted-auto" }

        runner.autonomousInit()
        runner.autonomousPeriodic()

        assertFalse(runner.isFaultedForTest)
        assertTrue(runner.isFinishedForTest)
        assertEquals("do-nothing", runner.selectedAutoForTest)
        assertEquals("Complete", runner.statusForTest)
        assertEquals(0.0, robot.store.state.drive.xVelocityMetersPerSecond)
        assertEquals(0.0, robot.store.state.drive.yVelocityMetersPerSecond)
        assertEquals(0.0, robot.store.state.drive.angularVelocityRadiansPerSecond)
    }

    @Test
    fun `stop cancels active generated routine and zeros outputs`() {
        val robot = newRobot(Alliance.BLUE)
        val runner = runner(robot) { "sim-drive-and-shoot" }
        runner.autonomousInit()
        runner.autonomousPeriodic()
        assertFalse(runner.isFinishedForTest)

        runner.stop()

        assertTrue(runner.isFinishedForTest)
        assertEquals("Stopped", runner.statusForTest)
        assertEquals(
            RoutineExecutionStatus.CANCELLED,
            robot.store.state.routineState.lastTerminalExecution?.status
        )
        assertEquals(0.0, robot.store.state.drive.xVelocityMetersPerSecond)
        assertFalse(robot.store.state.superstructure.marvin.flywheelActive)
    }

    @Test
    fun `generated readiness condition is fresh aligned and fail closed`() {
        val robot = newRobot(Alliance.BLUE)
        val ready = FrcAutoCapabilities.conditionShooterReady()
        robot.store.dispatch(SetFlywheelSpeed(4_000.0))
        robot.store.dispatch(sensorUpdate(valid = false))
        assertFalse(ready(robot.store.state))

        robot.store.dispatch(sensorUpdate(valid = true))
        assertTrue(ready(robot.store.state))

        val feedTask = FrcAutoCapabilities.actionShooterFeedWhenReady()
        feedTask.initialize(robot.store.state)
        feedTask.execute(robot.store.state, 20L).forEach(robot.store::dispatch)
        assertTrue(feedTask.isCompleted(robot.store.state, 20L))
        assertTrue(robot.store.state.superstructure.marvin.transferActive)
        assertNotNull(GeneratedAresProject.runtimeBindings(FrcAutoCapabilities))
    }

    private fun sensorUpdate(valid: Boolean) = SuperstructureSensorUpdate(
        flywheelRpm = 4_000.0,
        cowlAngleRotations = 0.0,
        intakeAngle = 0.0,
        pieceDetected = false,
        flywheelVelocityValid = valid
    )

    private fun newRobot(alliance: Alliance): FrcSwerveRobot = FrcSwerveRobot(
        isSimulation = true,
        initialState = RobotState(
            superstructure = SuperstructureState(custom = MarvinState())
        ),
        reducer = MarvinReducer::reduce
    ).also { robot -> robot.store.dispatch(RobotAction.SetAlliance(alliance)) }

    private fun runner(robot: FrcSwerveRobot, selection: () -> String) =
        FRCAutoOrchestrator(robot = robot, selectionProvider = selection)
}
