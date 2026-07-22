package com.areslib.frc.robot

import com.areslib.control.assist.ShotResult
import com.areslib.frc.FrcSwerveRobot
import com.areslib.frc.marvin.MarvinClimberSubsystem
import com.areslib.frc.marvin.MarvinIntakeSubsystem
import com.areslib.frc.marvin.MarvinShooterSubsystem
import com.areslib.frc.marvin.marvin
import com.areslib.frc.marvin.SetClimberVoltage
import com.areslib.frc.marvin.SetCowlAngle
import com.areslib.frc.marvin.SetFeederSpeed
import com.areslib.frc.marvin.SetFloorSpeed
import com.areslib.frc.marvin.SetFlywheelActive
import com.areslib.frc.marvin.SetFlywheelSpeed
import com.areslib.frc.marvin.SetIntakePivot
import com.areslib.frc.marvin.SetIntakeRollers
import com.areslib.frc.marvin.StartSlamtake
import com.areslib.frc.marvin.StopSlamtake
import com.areslib.math.geometry.Translation2d
import com.areslib.telemetry.GamepadState
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
/**
 * Documentation for FRCTeleOpDriveController
 */

class FRCTeleOpDriveController(
    private val robot: FrcSwerveRobot,
    private val marvinShooter: MarvinShooterSubsystem,
    private val marvinIntake: MarvinIntakeSubsystem,
    private val marvinClimber: MarvinClimberSubsystem,
    private val controller: XboxController,
    private val coPilotController: XboxController,
    private val controllerState: GamepadState,
    private val coPilotControllerState: GamepadState
) {
    private var driverYawOffset = 0.0
    private var intakeDeployed = false
    private var lastBeached = false
    private val shotResult = ShotResult()

    // Pre-allocated objects
    private val targetPosesRed = arrayOf(Translation2d(14.6, 6.0), Translation2d(14.6, 2.0))
    private val targetPosesBlue = arrayOf(Translation2d(2.0, 6.0), Translation2d(2.0, 2.0))
    /**
     * Documentation for cachedAlliance
     */

    var cachedAlliance: DriverStation.Alliance = DriverStation.Alliance.Blue
    /**
     * Documentation for speakerTranslation
     */
    var speakerTranslation = Translation2d(4.625, 4.035) // Blue speaker default
    /**
     * Documentation for teleopInit
     */

    fun teleopInit() {
        driverYawOffset = 0.0
    }
    /**
     * Documentation for teleopPeriodic
     */

    fun teleopPeriodic() {
        try {
            /**
             * Documentation for marvin
             */
            val marvin = robot.store.state.superstructure.marvin
            /**
             * Documentation for rawForward
             */

            val rawForward = MathUtil.applyDeadband(-controllerState.leftStickY.toDouble(), 0.1) * 4.5
            /**
             * Documentation for rawStrafe
             */
            val rawStrafe = MathUtil.applyDeadband(-controllerState.leftStickX.toDouble(), 0.1) * 4.5
            
            // Rotate joystick translation inputs by driverYawOffset to make controls relative to the driver's reset heading
            /**
             * Documentation for rotCos
             */
            val rotCos = Math.cos(driverYawOffset)
            /**
             * Documentation for rotSin
             */
            val rotSin = Math.sin(driverYawOffset)
            /**
             * Documentation for forward
             */
            val forward = rawForward * rotCos - rawStrafe * rotSin
            /**
             * Documentation for strafe
             */
            val strafe = rawForward * rotSin + rawStrafe * rotCos
            /**
             * Documentation for rotation
             */
            
            var rotation = MathUtil.applyDeadband(-controllerState.rightStickX.toDouble(), 0.1) * Math.PI
            /**
             * Documentation for currentPose
             */

            val currentPose = robot.store.state.drive.poseEstimator.estimatedPose

            // ── Copilot Swerve Lock Override ──
            if (coPilotControllerState.x) {
                robot.drive.joystickDrive(0.0, 0.0, 0.0, isXLock = true)
                return
            }

            // ── Gyro Reset (Driver Coordinate Alignment) ──
            if (controllerState.back || coPilotControllerState.back) {
                driverYawOffset = robot.store.state.drive.odometryHeading
            }

            // ── Driver / Copilot Shooting Triggers ──
            /**
             * Documentation for rtPressed
             */
            val rtPressed = controllerState.rightTrigger > 0.5f
            /**
             * Documentation for rbPressed
             */
            val rbPressed = controllerState.rightBumper
            /**
             * Documentation for bPressed
             */
            val bPressed = controllerState.b
            /**
             * Documentation for copilotRtPressed
             */
            val copilotRtPressed = coPilotControllerState.rightTrigger > 0.5f
            /**
             * Documentation for copilotRbPressed
             */
            val copilotRbPressed = coPilotControllerState.rightBumper
            /**
             * Documentation for targetFlywheelActive
             */
            var targetFlywheelActive = false
            /**
             * Documentation for targetFlywheelSpeed
             */
            var targetFlywheelSpeed = marvin.flywheel.targetVelocityRpm
            /**
             * Documentation for targetCowlAngle
             */
            var targetCowlAngle = marvin.cowl.targetAngleRotations

            rotation = when {
                rtPressed -> {
                    // Shoot-on-the-Move (SOTM) Speaker Aiming
                    marvinShooter.updateShootOnTheMove(
                        currentPose = currentPose,
                        targetTranslation = speakerTranslation,
                        shotResult = shotResult
                    )
                }
                rbPressed -> {
                    // Aim and Shuttle
                    /**
                     * Documentation for isRed
                     */
                    val isRed = cachedAlliance == DriverStation.Alliance.Red
                    /**
                     * Documentation for shuttleTarget
                     */
                    val shuttleTarget = if (isRed) targetPosesRed[1] else targetPosesBlue[1]

                    marvinShooter.updateShootOnTheMove(
                        currentPose = currentPose,
                        targetTranslation = shuttleTarget,
                        shotResult = shotResult,
                        runFloorRollers = true
                    )
                }
                bPressed -> {
                    // Static Shoot (Speaker Aiming)
                    marvinShooter.updateStaticShoot(
                        currentPose = currentPose,
                        targetTranslation = speakerTranslation
                    )
                }
                else -> rotation
            }

            when {
                copilotRtPressed -> {
                    targetFlywheelActive = true
                    targetFlywheelSpeed = 3350.0
                    targetCowlAngle = 0.5
                }
                copilotRbPressed -> {
                    targetFlywheelActive = true
                    targetFlywheelSpeed = 3650.0
                    targetCowlAngle = 1.1
                }
                else -> {
                    targetFlywheelActive = false
                }
            }

            // Dispatch flywheel & cowl changes only
            if (!rtPressed && !rbPressed && !bPressed) {
                /**
                 * Documentation for currentFlywheelActive
                 */
                val currentFlywheelActive = robot.store.state.superstructure.marvin.flywheelActive
                if (currentFlywheelActive != targetFlywheelActive) {
                    robot.store.dispatch(SetFlywheelActive(targetFlywheelActive, com.areslib.util.RobotClock.currentTimeMillis()))
                }
                if (targetFlywheelActive) {
                    if (marvin.flywheel.targetVelocityRpm != targetFlywheelSpeed) {
                        robot.store.dispatch(SetFlywheelSpeed(targetFlywheelSpeed))
                    }
                    if (marvin.cowl.targetAngleRotations != targetCowlAngle) {
                        robot.store.dispatch(SetCowlAngle(targetCowlAngle))
                    }
                }
            }

            // Apply drive command
            robot.drive.joystickDrive(forward, strafe, rotation, isFieldCentric = true)

            // ── A Button: Start Slamtake Sequence ──
            /**
             * Documentation for aPressed
             */
            val aPressed = controllerState.a
            /**
             * Documentation for isSlamtakeActive
             */
            val isSlamtakeActive = robot.store.state.superstructure.marvin.slamtakeActive
            if (aPressed && !isSlamtakeActive) {
                robot.store.dispatch(StartSlamtake())
            }

            // ── Left Bumper: Unjam ──
            /**
             * Documentation for lbPressed
             */
            val lbPressed = controllerState.leftBumper

            // ── Left Trigger: Intake/Feeder active run ──
            /**
             * Documentation for ltPressed
             */
            val ltPressed = controllerState.leftTrigger > 0.5f
            /**
             * Documentation for copilotLtPressed
             */
            val copilotLtPressed = coPilotControllerState.leftTrigger > 0.5f

            // ── POV Left/Right: Manual Intake Deploy Override ──
            when {
                controllerState.dpadRight -> intakeDeployed = true
                controllerState.dpadLeft -> intakeDeployed = false
            }

            // Dispatch states according to pilot control priorities
            /**
             * Documentation for targetPivot
             */
            var targetPivot = intakeDeployed
            /**
             * Documentation for targetIntakeRollers
             */
            var targetIntakeRollers = 0.0
            /**
             * Documentation for targetFloorSpeed
             */
            var targetFloorSpeed = 0.0
            /**
             * Documentation for targetFeederSpeed
             */
            var targetFeederSpeed = 0.0

            when {
                lbPressed -> {
                    // Unjam sequence takes top priority
                    if (isSlamtakeActive) {
                        robot.store.dispatch(StopSlamtake())
                    }
                    targetPivot = true
                    targetIntakeRollers = -5.0
                    targetFloorSpeed = -5.0
                    targetFeederSpeed = -5.0
                }
                isSlamtakeActive -> {
                    // Handled inside MarvinReducer! We do not mutate targets manually here
                }
                ltPressed -> {
                    // Active manual intake
                    targetPivot = true
                    targetIntakeRollers = 10.0
                    targetFloorSpeed = 10.0
                    targetFeederSpeed = 10.0
                }
                copilotLtPressed -> {
                    // Copilot manual feed override
                    targetPivot = intakeDeployed
                    targetIntakeRollers = 10.0
                    targetFloorSpeed = 10.0
                    targetFeederSpeed = 10.0
                }
                else -> {
                    // Default stop everything
                    targetPivot = intakeDeployed
                    targetIntakeRollers = 0.0
                    targetFloorSpeed = 0.0
                    if (!rtPressed && !rbPressed && !bPressed) {
                        targetFeederSpeed = 0.0
                    } else {
                        targetFeederSpeed = marvin.feeder.targetVelocityRps
                    }
                }
            }

            // Only dispatch changes to avoid hot-path Redux allocations
            if (!isSlamtakeActive) {
                if (marvin.intake.isDeployed != targetPivot) {
                    robot.store.dispatch(SetIntakePivot(deployed = targetPivot))
                }
                if (marvin.intake.targetRollerVelocityRps != targetIntakeRollers) {
                    robot.store.dispatch(SetIntakeRollers(targetIntakeRollers))
                }
                if (marvin.floor.targetVelocityRps != targetFloorSpeed) {
                    robot.store.dispatch(SetFloorSpeed(targetFloorSpeed))
                }
                if (marvin.feeder.targetVelocityRps != targetFeederSpeed) {
                    robot.store.dispatch(SetFeederSpeed(targetFeederSpeed))
                }
            }

            // ── POV Up/Down: Climber Voltage (Driver or Copilot) ──
            /**
             * Documentation for povUp
             */
            val povUp = controllerState.dpadUp || coPilotControllerState.dpadUp
            /**
             * Documentation for povDown
             */
            val povDown = controllerState.dpadDown || coPilotControllerState.dpadDown
            /**
             * Documentation for targetClimberVoltage
             */
            val targetClimberVoltage = when {
                povUp -> 6.0
                povDown -> -6.0
                else -> 0.0
            }
            if (marvin.climber.targetVoltage != targetClimberVoltage) {
                robot.store.dispatch(SetClimberVoltage(targetClimberVoltage))
            }

            // ── Beach / Traction Loss detection ──
            /**
             * Documentation for beached
             */
            val beached = robot.isBeached
            robot.telemetry.putBoolean("Diagnostics/Beached", beached)
            if (beached != lastBeached) {
                lastBeached = beached
                if (beached) {
                    controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
                    coPilotController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
                } else {
                    controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
                    coPilotController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
                }
            }
        } catch (e: Throwable) {
            System.err.println("ARESRobot: Exception in teleopPeriodic: ${e.message}")
            e.printStackTrace()
            robot.safeHardware()
        }
    }
}
