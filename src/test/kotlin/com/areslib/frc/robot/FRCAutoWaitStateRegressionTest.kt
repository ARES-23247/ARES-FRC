package com.areslib.frc.robot

import com.areslib.action.RobotAction
import com.areslib.frc.Dyn4jSimulation
import com.areslib.frc.FrcSwerveRobot
import com.areslib.frc.PathLoader
import com.areslib.frc.marvin.MarvinIntakeSubsystem
import com.areslib.frc.marvin.MarvinReducer
import com.areslib.frc.marvin.MarvinShooterSubsystem
import com.areslib.frc.marvin.MarvinState
import com.areslib.frc.marvin.marvin
import com.areslib.pathing.MutablePathPoint
import com.areslib.pathing.Path
import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import com.areslib.util.RobotClock
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

class FRCAutoWaitStateRegressionTest {

    private lateinit var robot: FrcSwerveRobot
    private lateinit var orchestrator: FRCAutoOrchestrator

    @BeforeEach
    fun setUp() {
        RobotClock.useMockTime(1_000L)
        robot = FrcSwerveRobot(
            isSimulation = true,
            initialState = RobotState(
                superstructure = SuperstructureState(custom = MarvinState())
            ),
            reducer = { state, action -> MarvinReducer.reduce(state, action) }
        )
        orchestrator = FRCAutoOrchestrator(
            robot,
            Dyn4jSimulation(seed = 42L),
            MarvinShooterSubsystem(robot.store),
            MarvinIntakeSubsystem(robot.store)
        )
    }

    @AfterEach
    fun tearDown() {
        RobotClock.useSystemTime()
    }

    @Test
    fun `FeederShoot freezes profile through wait then resumes after timeout without firing`() {
        orchestrator.autonomousInit()
        val path = PathLoader.loadPath("SimPath")
        val marker = path.events.first { it.eventName == "FeederShoot" }
        val markerPoint = MutablePathPoint()
        path.sampleAtDistance(marker.triggerDistanceMeters, markerPoint)

        // Keep this test independent of global DriverStation alliance state by installing
        // the known blue-origin fixture after autonomous initialization.
        setPrivateField("activePath", path)
        setPrivateDouble("actualPathDistance", marker.triggerDistanceMeters)
        robot.store.dispatch(
            RobotAction.PoseUpdate(
                xMeters = markerPoint.x,
                yMeters = markerPoint.y,
                headingRadians = markerPoint.headingRad,
                timestampMs = 1_000L,
                isReset = true
            )
        )

        RobotClock.setMockTimeMs(1_020L)
        orchestrator.autonomousPeriodic()
        assertEquals(marker.triggerDistanceMeters, orchestrator.targetDistanceMetersForTest, 1e-6)
        assertEquals(0.0, robot.store.state.superstructure.marvin.feeder.targetVelocityRps)

        RobotClock.setMockTimeMs(3_019L)
        orchestrator.autonomousPeriodic()
        assertEquals(
            marker.triggerDistanceMeters,
            orchestrator.targetDistanceMetersForTest,
            1e-6,
            "Profile distance must stay pinned while the command is still inside its two-second wait"
        )

        RobotClock.setMockTimeMs(3_021L)
        orchestrator.autonomousPeriodic()
        assertEquals(marker.triggerDistanceMeters, orchestrator.targetDistanceMetersForTest, 1e-6)
        assertEquals(0.0, robot.store.state.superstructure.marvin.feeder.targetVelocityRps)
        assertFalse(robot.store.state.superstructure.marvin.transferActive)
        assertFalse(orchestrator.isFaultedForTest)

        RobotClock.setMockTimeMs(3_041L)
        orchestrator.autonomousPeriodic()
        assertTrue(
            orchestrator.targetDistanceMetersForTest > marker.triggerDistanceMeters,
            "The time profile must resume on the loop after the bounded wait completes"
        )
    }

    private fun setPrivateField(name: String, value: Path) {
        FRCAutoOrchestrator::class.java.getDeclaredField(name).apply {
            isAccessible = true
            set(orchestrator, value)
        }
    }

    private fun setPrivateDouble(name: String, value: Double) {
        FRCAutoOrchestrator::class.java.getDeclaredField(name).apply {
            isAccessible = true
            setDouble(orchestrator, value)
        }
    }
}
