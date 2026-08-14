package com.areslib.frc

import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj.simulation.XboxControllerSim
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.*

class ARESRobotTest {

    private companion object {
        const val LEFT_BUMPER_BUTTON = 5
        const val RIGHT_BUMPER_BUTTON = 6
    }

    private lateinit var robot: ARESRobot
    private lateinit var controllerSim: XboxControllerSim
    private lateinit var coPilotSim: XboxControllerSim

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500, 0))
        DriverStationSim.setEnabled(true)
        controllerSim = XboxControllerSim(0)
        coPilotSim = XboxControllerSim(1)
        controllerSim.setPOV(-1)
        coPilotSim.setPOV(-1)
        robot = ARESRobot()
    }

    @AfterEach
    fun tearDown() {
        robot.close()
    }

    @Test
    fun testRobotLifecycle() {
        robot.robotInit()

        // Test disabled
        DriverStationSim.setAutonomous(false)
        DriverStationSim.setEnabled(false)
        robot.robotPeriodic()
        robot.disabledInit()
        robot.disabledPeriodic()

        // Test autonomous
        DriverStationSim.setAutonomous(true)
        DriverStationSim.setEnabled(true)
        robot.autonomousInit()
        robot.autonomousPeriodic()
        robot.robotPeriodic()

        // Test teleop init
        DriverStationSim.setAutonomous(false)
        DriverStationSim.setEnabled(true)
        robot.teleopInit()
        robot.teleopPeriodic()
        robot.robotPeriodic()

        // Test various button configurations in teleop to cover all branches:
        // 1. backButton -> reset gyro
        controllerSim.setBackButton(true)
        robot.teleopPeriodic()
        controllerSim.setBackButton(false)

        // 2. xButton on copilot -> lock swerve
        coPilotSim.setXButton(true)
        robot.teleopPeriodic()
        coPilotSim.setXButton(false)

        // 3. rightTriggerAxis -> SOTM
        controllerSim.setRightTriggerAxis(0.8)
        robot.teleopPeriodic()
        controllerSim.setRightTriggerAxis(0.0)

        // 4. rightBumper -> Shuttle
        controllerSim.setRawButton(RIGHT_BUMPER_BUTTON, true)
        robot.teleopPeriodic()
        controllerSim.setRawButton(RIGHT_BUMPER_BUTTON, false)

        // 5. bButton -> static shoot
        controllerSim.setBButton(true)
        robot.teleopPeriodic()
        controllerSim.setBButton(false)

        // 6. copilot rt -> flywheel low speed
        coPilotSim.setRightTriggerAxis(0.8)
        robot.teleopPeriodic()
        coPilotSim.setRightTriggerAxis(0.0)

        // 7. copilot rb -> flywheel high speed
        coPilotSim.setRawButton(RIGHT_BUMPER_BUTTON, true)
        robot.teleopPeriodic()
        coPilotSim.setRawButton(RIGHT_BUMPER_BUTTON, false)

        // 8. aButton -> Start Slamtake
        controllerSim.setAButton(true)
        robot.teleopPeriodic()
        controllerSim.setAButton(false)

        // 9. leftBumper -> Unjam
        controllerSim.setRawButton(LEFT_BUMPER_BUTTON, true)
        robot.teleopPeriodic()
        controllerSim.setRawButton(LEFT_BUMPER_BUTTON, false)
    }

    @Test
    fun testTestModeLifecycleAndDisabledTriggers() {
        robot.robotInit()

        // ── 1. Disabled Mode & Gyro Reset / Homing Trigger Handling ──
        DriverStationSim.setAutonomous(false)
        DriverStationSim.setTest(false)
        DriverStationSim.setEnabled(false)
        DriverStationSim.notifyNewData()

        robot.disabledInit()
        robot.robotPeriodic()
        robot.disabledPeriodic()

        // Individual button presses (non-combo) in disabled mode
        controllerSim.setBackButton(true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.disabledPeriodic()
        controllerSim.setBackButton(false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.disabledPeriodic()

        controllerSim.setStartButton(true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.disabledPeriodic()
        controllerSim.setStartButton(false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.disabledPeriodic()

        coPilotSim.setBackButton(true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.disabledPeriodic()
        coPilotSim.setBackButton(false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.disabledPeriodic()

        coPilotSim.setStartButton(true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.disabledPeriodic()
        coPilotSim.setStartButton(false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.disabledPeriodic()

        // Dual-operator Back + Start homing trigger in disabled mode
        controllerSim.setBackButton(true)
        controllerSim.setStartButton(true)
        coPilotSim.setBackButton(true)
        coPilotSim.setStartButton(true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.disabledPeriodic()

        controllerSim.setBackButton(false)
        controllerSim.setStartButton(false)
        coPilotSim.setBackButton(false)
        coPilotSim.setStartButton(false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.disabledPeriodic()

        // ── 2. Test Mode Lifecycle (testInit, testPeriodic, testExit) ──
        DriverStationSim.setTest(true)
        DriverStationSim.setEnabled(true)
        DriverStationSim.notifyNewData()

        robot.testInit()
        robot.robotPeriodic()
        robot.testPeriodic()

        assertEquals("VISION_STATIONARY", edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString("Calibration/Localization/TestType", ""))
        assertFalse(edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean("Calibration/Localization/Recording", true))
        assertEquals(0.0, edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Calibration/Localization/TruthX", -1.0), 1e-6)
        assertEquals(0.0, edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Calibration/Localization/TruthY", -1.0), 1e-6)
        assertEquals(0.0, edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Calibration/Localization/TruthHeadingRad", -1.0), 1e-6)

        // Toggle continuous recording with 'A'
        controllerSim.setAButton(true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        assertTrue(edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean("Calibration/Localization/Recording", false))
        controllerSim.setAButton(false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()

        // Cycle test type with 'B'
        controllerSim.setBButton(true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        assertNotEquals("VISION_STATIONARY", edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString("Calibration/Localization/TestType", ""))
        controllerSim.setBButton(false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()

        // Mark start with 'X' and mark end with 'Y'
        controllerSim.setXButton(true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        controllerSim.setXButton(false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()

        controllerSim.setYButton(true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        controllerSim.setYButton(false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()

        // Adjust heading with Bumpers
        controllerSim.setRawButton(LEFT_BUMPER_BUTTON, true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        controllerSim.setRawButton(LEFT_BUMPER_BUTTON, false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()

        controllerSim.setRawButton(RIGHT_BUMPER_BUTTON, true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        controllerSim.setRawButton(RIGHT_BUMPER_BUTTON, false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()

        // Adjust truth with D-Pad POV
        controllerSim.setPOV(0)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        controllerSim.setPOV(180)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        controllerSim.setPOV(270)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        controllerSim.setPOV(90)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        controllerSim.setPOV(-1)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()

        // Zero truth with Back button
        controllerSim.setBackButton(true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        assertEquals(0.0, edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Calibration/Localization/TruthX", -1.0), 1e-6)
        assertEquals(0.0, edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Calibration/Localization/TruthY", -1.0), 1e-6)
        assertEquals(0.0, edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Calibration/Localization/TruthHeadingRad", -1.0), 1e-6)
        controllerSim.setBackButton(false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()

        // Seed pose to truth with Start button
        controllerSim.setStartButton(true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        controllerSim.setStartButton(false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()

        // Test drivePeriodic in test mode (joystick drive & xLock)
        controllerSim.setLeftY(-0.5)
        controllerSim.setLeftX(-0.5)
        controllerSim.setRightX(0.2)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        controllerSim.setLeftY(0.0)
        controllerSim.setLeftX(0.0)
        controllerSim.setRightX(0.0)
        DriverStationSim.notifyNewData()

        coPilotSim.setXButton(true)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()
        coPilotSim.setXButton(false)
        DriverStationSim.notifyNewData()
        robot.robotPeriodic()
        robot.testPeriodic()

        // Clean exit from test mode
        robot.testExit()
    }
}
