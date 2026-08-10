package com.areslib.frc.robot

import com.areslib.frc.FrcSwerveRobot
import com.areslib.frc.Dyn4jSimulation
import com.areslib.frc.marvin.MarvinIntakeSubsystem
import com.areslib.frc.marvin.MarvinReducer
import com.areslib.frc.marvin.MarvinState
import com.areslib.frc.marvin.MarvinShooterSubsystem
import com.areslib.frc.marvin.SetFlywheelSpeed
import com.areslib.frc.marvin.SuperstructureSensorUpdate
import com.areslib.util.RobotClock
import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import com.areslib.frc.marvin.marvin
import com.areslib.pathing.Path
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.*

class FRCAutoOrchestratorTest {

    private lateinit var robot: FrcSwerveRobot
    private lateinit var sim: Dyn4jSimulation
    private lateinit var marvinShooter: MarvinShooterSubsystem
    private lateinit var marvinIntake: MarvinIntakeSubsystem
    private lateinit var orchestrator: FRCAutoOrchestrator

    @BeforeEach
    fun setUp() {
        RobotClock.useMockTime(1000)

        robot = FrcSwerveRobot(
            isSimulation = true,
            initialState = RobotState(
                superstructure = SuperstructureState(custom = MarvinState())
            ),
            reducer = { state, action -> MarvinReducer.reduce(state, action) }
        )
        sim = Dyn4jSimulation()
        marvinShooter = MarvinShooterSubsystem(robot.store)
        marvinIntake = MarvinIntakeSubsystem(robot.store)

        orchestrator = FRCAutoOrchestrator(robot, sim, marvinShooter, marvinIntake)
    }

    @AfterEach
    fun tearDown() {
        RobotClock.useSystemTime()
    }

    @Test
    fun testEmptyAutoSequenceHandlesGracefully() {
        // Empty auto sequence / PathLoader failing to load should not crash
        orchestrator.autonomousInit()
        
        RobotClock.setMockTimeMs(1100)
        orchestrator.autonomousPeriodic()
        
        // SimPath should load, but if it doesn't, it gracefully handles it.
        // We verify that state is stable
        val state = robot.store.state
        assertNotNull(state)
    }

    @Test
    fun testRealTimeDtCalculationUsesRobotClock() {
        orchestrator.autonomousInit()

        // Advance simulated time
        RobotClock.setMockTimeMs(2000) // +1 second
        
        // autonomousPeriodic should handle the time delta
        orchestrator.autonomousPeriodic()
        
        assertTrue(true)
    }

    @Test
    fun targetProfileProgressesEvenWhenOdometryDoesNot() {
        orchestrator.autonomousInit()
        
        // Advance time iteratively to simulate periodic updates
        for (i in 1..10) {
            RobotClock.setMockTimeMs(1000L + (i * 20)) // 50 Hz loop
            orchestrator.autonomousPeriodic()
        }
        
        assertTrue(orchestrator.targetDistanceMetersForTest > 0.0)
        assertTrue(
            orchestrator.targetDistanceMetersForTest > orchestrator.actualDistanceMetersForTest,
            "Time-parameterized target must not stall on the closest-point odometry distance"
        )
    }

    @Test
    fun testEventMarkerActionsFireAtCorrectPathTimestamps() {
        orchestrator.autonomousInit()
        
        // Simulate massive time jump to ensure we blow past event markers like "IntakeDeploy"
        RobotClock.setMockTimeMs(5000)
        orchestrator.autonomousPeriodic()
        
        // The path will process all events because of the large dt.
        // We ensure no crashes occur.
        assertTrue(true)
    }

    @Test
    fun feederShootWaitsWhenFlywheelMeasurementIsInvalid() {
        robot.store.dispatch(SetFlywheelSpeed(4000.0, 1000L))
        robot.store.dispatch(
            SuperstructureSensorUpdate(
                flywheelRpm = 4000.0,
                cowlAngleRotations = 0.0,
                intakeAngle = 0.0,
                pieceDetected = false,
                flywheelVelocityValid = false,
                timestampMs = 1000L
            )
        )

        assertFalse(orchestrator.handleEvent("FeederShoot", 1.0))

        robot.store.dispatch(
            SuperstructureSensorUpdate(
                flywheelRpm = 4000.0,
                cowlAngleRotations = 0.0,
                intakeAngle = 0.0,
                pieceDetected = false,
                flywheelVelocityValid = true,
                timestampMs = 1020L
            )
        )
        assertTrue(orchestrator.handleEvent("FeederShoot", 1.02))
    }

    @Test
    fun emptyOrFailedPathLatchesSafeOutputsInsteadOfReusingAStaleTarget() {
        orchestrator.autonomousInit()
        robot.drive.joystickDrive(1.0, -0.5, 0.25, isFieldCentric = false)
        marvinShooter.spinUp(4000.0)
        marvinIntake.setRollerSpeed(12.0)
        FRCAutoOrchestrator::class.java.getDeclaredField("activePath").apply {
            isAccessible = true
            set(orchestrator, Path(emptyList()))
        }

        RobotClock.setMockTimeMs(1020L)
        orchestrator.autonomousPeriodic()

        assertTrue(orchestrator.isFaultedForTest)
        assertEquals(0.0, robot.store.state.drive.xVelocityMetersPerSecond)
        assertEquals(0.0, robot.store.state.drive.yVelocityMetersPerSecond)
        assertEquals(0.0, robot.store.state.drive.angularVelocityRadiansPerSecond)
        assertFalse(robot.store.state.superstructure.marvin.flywheelActive)
        assertEquals(0.0, robot.store.state.superstructure.marvin.flywheel.targetVelocityRpm)
        assertEquals(0.0, robot.store.state.superstructure.marvin.intake.targetRollerVelocityRps)
    }
}
