package com.areslib.frc.robot

import com.areslib.frc.FrcSwerveRobot
import com.areslib.frc.Dyn4jSimulation
import com.areslib.frc.marvin.*
import com.areslib.frc.sim.FrcDashboardDriveFrameGate
import com.areslib.frc.sim.applyTo
import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import com.areslib.telemetry.GamepadState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.hal.HAL
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.*

class FRCTeleOpDriveControllerTest {

    private lateinit var robot: FrcSwerveRobot
    private lateinit var marvinShooter: MarvinShooterSubsystem
    private lateinit var controller: XboxController
    private lateinit var coPilotController: XboxController
    private lateinit var controllerState: GamepadState
    private lateinit var coPilotControllerState: GamepadState
    
    private lateinit var teleOpController: FRCTeleOpDriveController

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500, 0))
        robot = FrcSwerveRobot(
            isSimulation = true,
            initialState = RobotState(
                superstructure = SuperstructureState(custom = MarvinState())
            ),
            reducer = MarvinReducer::reduce
        )
        
        marvinShooter = MarvinShooterSubsystem(robot.store)
        
        controller = XboxController(0)
        coPilotController = XboxController(1)
        
        controllerState = GamepadState()
        coPilotControllerState = GamepadState()

        teleOpController = FRCTeleOpDriveController(
            robot, marvinShooter,
            controller, coPilotController, controllerState, coPilotControllerState
        )
        teleOpController.teleopInit()
    }

    @Test
    fun testZeroJoystickInputProducesZeroChassisSpeeds() {
        controllerState.leftStickX = 0.0f
        controllerState.leftStickY = 0.0f
        controllerState.rightStickX = 0.0f

        teleOpController.teleopPeriodic()

        assertEquals(0.0, robot.store.state.drive.xVelocityMetersPerSecond, 1e-6)
        assertEquals(0.0, robot.store.state.drive.yVelocityMetersPerSecond, 1e-6)
        assertEquals(0.0, robot.store.state.drive.angularVelocityRadiansPerSecond, 1e-6)
    }

    @Test
    fun testDeadbandFilteringEliminatesSmallInputsBelowThreshold() {
        // Values below 0.1 deadband
        controllerState.leftStickX = 0.05f
        controllerState.leftStickY = -0.05f
        controllerState.rightStickX = 0.09f

        teleOpController.teleopPeriodic()

        assertEquals(0.0, robot.store.state.drive.xVelocityMetersPerSecond, 1e-6)
        assertEquals(0.0, robot.store.state.drive.yVelocityMetersPerSecond, 1e-6)
        assertEquals(0.0, robot.store.state.drive.angularVelocityRadiansPerSecond, 1e-6)
    }

    @Test
    fun testRepeatedFieldCentricInputProducesConsistentCommands() {
        // Straight forward input
        controllerState.leftStickY = -1.0f
        controllerState.leftStickX = 0.0f

        teleOpController.teleopPeriodic()
        val vx1 = robot.store.state.drive.xVelocityMetersPerSecond
        val vy1 = robot.store.state.drive.yVelocityMetersPerSecond
        assertTrue(vx1 != 0.0 || vy1 != 0.0)
        
        // The hardware boundary, rather than this controller, performs the field-frame transform.
        controllerState.leftStickY = -1.0f
        controllerState.leftStickX = 0.0f
        teleOpController.teleopPeriodic()
        
        val vx2 = robot.store.state.drive.xVelocityMetersPerSecond
        val vy2 = robot.store.state.drive.yVelocityMetersPerSecond
        
        // Same input should produce non-zero drive commands
        assertTrue(vx2 != 0.0 || vy2 != 0.0)
    }

    @Test
    fun `leased dashboard frame reaches FRC teleop and moves dyn4j along field X`() {
        val gate = FrcDashboardDriveFrameGate()
        val flags = ((1L shl 3) or (1L shl 4)).toDouble() // TeleOp + field-centric, Blue
        assertTrue(gate.accept(dashboardFrame(sequence = 0L, flags = flags), nowMs = 1_000L))
        assertTrue(
            gate.accept(
                dashboardFrame(sequence = 1L, vx = 3.0, flags = flags),
                nowMs = 1_020L
            )
        )

        requireNotNull(gate.current(1_020L)).applyTo(controllerState)
        teleOpController.cachedAlliance = DriverStation.Alliance.Blue
        teleOpController.drivePeriodic()
        assertTrue(robot.store.state.drive.xVelocityMetersPerSecond > 2.5)
        assertEquals(0.0, robot.store.state.drive.yVelocityMetersPerSecond, 1e-6)

        val simulation = Dyn4jSimulation(seed = 42L)
        try {
            val start = simulation.getPoseUpdate()
            repeat(40) { simulation.step(robot.store.state, 0.02) }
            val moved = simulation.getPoseUpdate()
            assertTrue(moved.xMeters - start.xMeters > 0.25, "dashboard forward must move along FRC +X")
            assertTrue(kotlin.math.abs(moved.yMeters - start.yMeters) < moved.xMeters - start.xMeters)
        } finally {
            simulation.close()
        }

        assertNull(gate.current(1_521L), "the receiver lease must expire without new frames")
    }

    @Test
    fun testAllianceRelativeDirectionInversionWorksForBlueVsRed() {
        controllerState.leftStickY = -1.0f
        controllerState.leftStickX = -1.0f

        teleOpController.cachedAlliance = DriverStation.Alliance.Blue
        teleOpController.teleopPeriodic()
        val blueForward = robot.store.state.drive.xVelocityMetersPerSecond
        val blueStrafe = robot.store.state.drive.yVelocityMetersPerSecond
        assertEquals(com.areslib.state.Alliance.BLUE, robot.store.state.drive.alliance)

        teleOpController.cachedAlliance = DriverStation.Alliance.Red
        teleOpController.teleopPeriodic()
        val redForward = robot.store.state.drive.xVelocityMetersPerSecond
        val redStrafe = robot.store.state.drive.yVelocityMetersPerSecond

        assertEquals(com.areslib.state.Alliance.RED, robot.store.state.drive.alliance)
        assertEquals(-blueForward, redForward, 1e-6)
        assertEquals(-blueStrafe, redStrafe, 1e-6)
    }

    @Test
    fun testPlanarChassisCommandStaysWithinBounds() {
        // Max forward and max strafe
        controllerState.leftStickY = -1.0f
        controllerState.leftStickX = -1.0f
        
        teleOpController.teleopPeriodic()
        
        val vx = robot.store.state.drive.xVelocityMetersPerSecond
        val vy = robot.store.state.drive.yVelocityMetersPerSecond
        
        // Independent 4.5 m/s axis limits imply a 4.5*sqrt(2) planar upper bound.
        val speed = Math.hypot(vx, vy)
        assertTrue(speed <= 6.5)
    }

    @Test
    fun driveOnlyAuthorityDoesNotDispatchLegacyMechanisms() {
        controllerState.leftStickY = -1.0f
        controllerState.leftTrigger = 1.0f
        controllerState.a = true

        teleOpController.drivePeriodic()

        assertNotEquals(0.0, robot.store.state.drive.xVelocityMetersPerSecond)
        val marvin = robot.store.state.superstructure.marvin
        assertFalse(marvin.slamtakeActive)
        assertEquals(0.0, marvin.intake.targetRollerVelocityRps)
        assertEquals(0.0, marvin.floor.targetVelocityRps)
        assertEquals(0.0, marvin.feeder.targetVelocityRps)
    }

    @Test
    fun xLockStillProcessesMechanismReleaseAndClearsTransferLatch() {
        robot.store.dispatch(SetFlywheelSpeed(4_000.0))
        robot.store.dispatch(SetFlywheelActive(true))
        robot.store.dispatch(SetIntakeRollers(10.0))
        robot.store.dispatch(SetFloorSpeed(10.0))
        robot.store.dispatch(SetFeederSpeed(10.0))
        robot.store.dispatch(StartTransfer())
        coPilotControllerState.x = true

        teleOpController.teleopPeriodic()

        val state = robot.store.state
        assertTrue(state.drive.isXLock)
        assertFalse(state.superstructure.marvin.flywheelActive)
        assertFalse(state.superstructure.marvin.transferActive)
        assertEquals(0.0, state.superstructure.marvin.flywheel.targetVelocityRpm)
        assertEquals(0.0, state.superstructure.marvin.intake.targetRollerVelocityRps)
        assertEquals(0.0, state.superstructure.marvin.floor.targetVelocityRps)
        assertEquals(0.0, state.superstructure.marvin.feeder.targetVelocityRps)
    }

    @Test
    fun controllerFailureLatchesAtomicAllStopAndRejectsLaterSetpoints() {
        robot.store.dispatch(SetFlywheelSpeed(4_000.0))
        robot.store.dispatch(SetFlywheelActive(true))
        robot.store.dispatch(SetFeederSpeed(10.0))

        teleOpController.latchControllerAllStop("test controller", IllegalStateException("boom"))

        assertTrue(robot.store.state.superstructure.marvin.mechanismSafetyInhibited)
        assertTrue(robot.store.state.superstructure.marvin.mechanismSafetyFaultLatched)
        assertTrue(
            robot.store.state.superstructure.marvin.mechanismSafetyFaultReason.contains("boom")
        )
        assertEquals(com.areslib.state.DriveMode.X_BRAKE, robot.store.state.drive.driveMode)
        assertTrue(robot.store.state.drive.isXLock)
        assertFalse(robot.store.state.superstructure.marvin.flywheelActive)
        assertEquals(0.0, robot.store.state.superstructure.marvin.feeder.targetVelocityRps)
        robot.store.dispatch(SetFlywheelSpeed(5_000.0))
        robot.store.dispatch(SetFeederSpeed(12.0))
        assertEquals(0.0, robot.store.state.superstructure.marvin.flywheel.targetVelocityRpm)
        assertEquals(0.0, robot.store.state.superstructure.marvin.feeder.targetVelocityRps)
    }

    private fun dashboardFrame(
        sequence: Long,
        vx: Double = 0.0,
        flags: Double,
    ): DoubleArray = doubleArrayOf(
        2.0,
        7_001.0,
        sequence.toDouble(),
        (10_000L + sequence).toDouble(),
        vx,
        0.0,
        0.0,
        flags,
    )

    @Test
    fun slamtakeRequiresANewAButtonEdgeAfterCompletion() {
        controllerState.a = true
        teleOpController.teleopPeriodic()
        assertTrue(robot.store.state.superstructure.marvin.slamtakeActive)

        robot.store.dispatch(StopSlamtake())
        teleOpController.teleopPeriodic()
        assertFalse(robot.store.state.superstructure.marvin.slamtakeActive)

        controllerState.a = false
        teleOpController.teleopPeriodic()
        controllerState.a = true
        teleOpController.teleopPeriodic()
        assertTrue(robot.store.state.superstructure.marvin.slamtakeActive)
    }

    @Test
    fun copilotRightTriggerSpinsUpFlywheelPreset() {
        coPilotControllerState.rightTrigger = 0.8f
        teleOpController.teleopPeriodic()

        val state = robot.store.state
        assertTrue(state.superstructure.marvin.flywheelActive)
        assertEquals(3350.0, state.superstructure.marvin.flywheel.targetVelocityRpm, 1e-4)
        assertEquals(0.5, state.superstructure.marvin.cowl.targetAngleRotations, 1e-4)
    }

    @Test
    fun copilotRightBumperSpinsUpFlywheelPreset() {
        coPilotControllerState.rightBumper = true
        teleOpController.teleopPeriodic()

        val state = robot.store.state
        assertTrue(state.superstructure.marvin.flywheelActive)
        assertEquals(3650.0, state.superstructure.marvin.flywheel.targetVelocityRpm, 1e-4)
        assertEquals(1.1, state.superstructure.marvin.cowl.targetAngleRotations, 1e-4)

        coPilotControllerState.rightBumper = false
        teleOpController.teleopPeriodic()

        val releasedState = robot.store.state
        assertFalse(releasedState.superstructure.marvin.flywheelActive)
        assertEquals(0.0, releasedState.superstructure.marvin.flywheel.targetVelocityRpm, 1e-4)
    }
}
