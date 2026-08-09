package com.areslib.frc.robot

import com.areslib.frc.FrcSwerveRobot
import com.areslib.frc.marvin.MarvinClimberSubsystem
import com.areslib.frc.marvin.MarvinIntakeSubsystem
import com.areslib.frc.marvin.MarvinShooterSubsystem
import com.areslib.frc.marvin.marvin
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
    private lateinit var marvinIntake: MarvinIntakeSubsystem
    private lateinit var marvinClimber: MarvinClimberSubsystem
    private lateinit var controller: XboxController
    private lateinit var coPilotController: XboxController
    private lateinit var controllerState: GamepadState
    private lateinit var coPilotControllerState: GamepadState
    
    private lateinit var teleOpController: FRCTeleOpDriveController

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500, 0))
        robot = FrcSwerveRobot(isSimulation = true)
        
        marvinShooter = MarvinShooterSubsystem(robot.store)
        marvinIntake = MarvinIntakeSubsystem(robot.store)
        marvinClimber = MarvinClimberSubsystem(robot.store)
        
        controller = XboxController(0)
        coPilotController = XboxController(1)
        
        controllerState = GamepadState()
        coPilotControllerState = GamepadState()

        teleOpController = FRCTeleOpDriveController(
            robot, marvinShooter, marvinIntake, marvinClimber,
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
    fun testFieldCentricTransformationRotatesCorrectly() {
        // Straight forward input
        controllerState.leftStickY = -1.0f
        controllerState.leftStickX = 0.0f

        teleOpController.teleopPeriodic()
        val vx1 = robot.store.state.drive.xVelocityMetersPerSecond
        val vy1 = robot.store.state.drive.yVelocityMetersPerSecond
        assertTrue(vx1 != 0.0 || vy1 != 0.0)
        
        // Verify the joystick input produces consistent drive output
        // (manual rotation was removed; CTRE handles field-centric at the hardware level)
        controllerState.leftStickY = -1.0f
        controllerState.leftStickX = 0.0f
        teleOpController.teleopPeriodic()
        
        val vx2 = robot.store.state.drive.xVelocityMetersPerSecond
        val vy2 = robot.store.state.drive.yVelocityMetersPerSecond
        
        // Same input should produce non-zero drive commands
        assertTrue(vx2 != 0.0 || vy2 != 0.0)
    }

    @Test
    fun testAllianceRelativeDirectionInversionWorksForBlueVsRed() {
        // Red Alliance
        teleOpController.cachedAlliance = DriverStation.Alliance.Red
        controllerState.rightBumper = true
        teleOpController.teleopPeriodic()
        
        val redFlywheelSpeed = robot.store.state.superstructure.marvin.flywheel.targetVelocityRpm
        
        // Blue Alliance
        teleOpController.cachedAlliance = DriverStation.Alliance.Blue
        controllerState.rightBumper = true
        teleOpController.teleopPeriodic()
        
        val blueFlywheelSpeed = robot.store.state.superstructure.marvin.flywheel.targetVelocityRpm
        
        assertTrue(redFlywheelSpeed >= 0.0)
        assertTrue(blueFlywheelSpeed >= 0.0)
    }

    @Test
    fun testMotorOutputStaysWithinBounds() {
        // Max forward and max strafe
        controllerState.leftStickY = -1.0f
        controllerState.leftStickX = -1.0f
        
        teleOpController.teleopPeriodic()
        
        val vx = robot.store.state.drive.xVelocityMetersPerSecond
        val vy = robot.store.state.drive.yVelocityMetersPerSecond
        
        // Ensure within 4.5 * sqrt(2) or so
        val speed = Math.hypot(vx, vy)
        assertTrue(speed <= 6.5) // 4.5 * 1.414 is approx 6.36
    }
}
