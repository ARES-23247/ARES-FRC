package com.areslib.frc.robot

import com.areslib.control.assist.ShotResult
import com.areslib.action.RobotAction
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
 * Converts the two cached Xbox-controller snapshots into Redux setpoints and a field-relative
 * swerve command during teleop.
 *
 * Translational inputs are meters per second in the blue-origin WPILib field frame; heading and
 * angular velocity are CCW-positive radians. Red-alliance translation is rotated by 180 degrees
 * so driver-forward remains away from the alliance wall. The controller dispatches only changed
 * setpoints on the 20 ms robot loop and never reads mechanism hardware directly.
 *
 * Command priority is intentionally explicit: copilot X-lock returns before mechanism handling;
 * driver shooting owns automatic aim; unjam overrides slamtake and manual intake; an active
 * slamtake owns its mechanism targets until the reducer ends it.
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
    private var intakeDeployed = false
    private var lastBeached = false
    private var rumbleStartTimestampMs: Long = 0
    private val shotResult = ShotResult()

    // Pre-allocated shuttle targets in blue-origin field meters; index 1 is currently selected.
    private val targetPosesRed = arrayOf(Translation2d(14.6, 6.0), Translation2d(14.6, 2.0))
    private val targetPosesBlue = arrayOf(Translation2d(2.0, 6.0), Translation2d(2.0, 2.0))

    /**
     * Alliance used to interpret field-relative driver input.
     *
     * Assigning this also keeps the shared Redux drive state synchronized with Driver Station
     * state, avoiding an alliance mismatch between input handling and other drive consumers.
     */
    var cachedAlliance: DriverStation.Alliance = DriverStation.Alliance.Blue
        set(value) {
            field = value
            val reduxAlliance = when (value) {
                DriverStation.Alliance.Red -> com.areslib.state.Alliance.RED
                DriverStation.Alliance.Blue -> com.areslib.state.Alliance.BLUE
            }
            if (robot.store.state.drive.alliance != reduxAlliance) {
                robot.store.dispatch(RobotAction.SetAlliance(reduxAlliance))
            }
        }
    /** Active alliance speaker target in blue-origin field coordinates, in meters. */
    var speakerTranslation = com.areslib.frc.marvin.MarvinConfig.FieldTargets.blueSpeaker

    /** Lifecycle hook retained for symmetry with [teleopPeriodic]; no reset is currently needed. */
    fun teleopInit() {
    }

    /**
     * Runs only the legacy drivetrain owner for a generated mechanism-control scheme.
     *
     * This deliberately excludes shooting assists and every mechanism dispatch. A generated
     * routine containing a drive step suppresses this method at the ARESRobot boundary, leaving
     * the path follower as the sole drivetrain owner for that frame.
     */
    fun drivePeriodic() {
        try {
            if (coPilotControllerState.x) {
                robot.drive.joystickDrive(0.0, 0.0, 0.0, isXLock = true)
                return
            }
            val allianceScale = if (cachedAlliance == DriverStation.Alliance.Red) -1.0 else 1.0
            val forward = MathUtil.applyDeadband(-controllerState.leftStickY.toDouble(), 0.1) *
                4.5 * allianceScale
            val strafe = MathUtil.applyDeadband(-controllerState.leftStickX.toDouble(), 0.1) *
                4.5 * allianceScale
            val rotation = MathUtil.applyDeadband(-controllerState.rightStickX.toDouble(), 0.1) * Math.PI
            robot.drive.joystickDrive(forward, strafe, rotation, isFieldCentric = true)
        } catch (error: Exception) {
            DriverStation.reportError("Exception in drivePeriodic: ${error.message}", false)
            robot.safeHardware()
        }
    }

    /** Processes one cached 20 ms input snapshot and emits drive/mechanism setpoints. */
    fun teleopPeriodic() {
        try {
            val marvin = robot.store.state.superstructure.marvin

            val rawForward = MathUtil.applyDeadband(-controllerState.leftStickY.toDouble(), 0.1) * 4.5
            val rawStrafe = MathUtil.applyDeadband(-controllerState.leftStickX.toDouble(), 0.1) * 4.5
            
            // Field coordinates are blue-origin. Rotate translation intent 180 degrees
            // on red so pushing away from either alliance wall remains driver-forward.
            val allianceScale = if (cachedAlliance == DriverStation.Alliance.Red) -1.0 else 1.0
            val forward = rawForward * allianceScale
            val strafe = rawStrafe * allianceScale
            
            var rotation = MathUtil.applyDeadband(-controllerState.rightStickX.toDouble(), 0.1) * Math.PI

            val currentPose = robot.store.state.drive.poseEstimator.estimatedPose

            // ── Copilot Swerve Lock Override ──
            if (coPilotControllerState.x) {
                robot.drive.joystickDrive(0.0, 0.0, 0.0, isXLock = true)
                return
            }

            // ── Driver / Copilot Shooting Triggers ──
            val rtPressed = controllerState.rightTrigger > 0.5f
            val rbPressed = controllerState.rightBumper
            val bPressed = controllerState.b
            val copilotRtPressed = coPilotControllerState.rightTrigger > 0.5f
            val copilotRbPressed = coPilotControllerState.rightBumper
            var targetFlywheelSpeed = marvin.flywheel.targetVelocityRpm
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
                    val isRed = cachedAlliance == DriverStation.Alliance.Red
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

            val targetFlywheelActive = when {
                copilotRtPressed -> {
                    targetFlywheelSpeed = 3350.0
                    targetCowlAngle = 0.5
                    true
                }
                copilotRbPressed -> {
                    targetFlywheelSpeed = 3650.0
                    targetCowlAngle = 1.1
                    true
                }
                else -> false
            }

            // Dispatch flywheel & cowl changes only
            if (!rtPressed && !rbPressed && !bPressed) {
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
                } else if (marvin.flywheel.targetVelocityRpm != 0.0) {
                    // Releasing the spin-up buttons must zero the target, otherwise the
                    // flywheel keeps spinning at the last commanded RPM indefinitely.
                    robot.store.dispatch(SetFlywheelSpeed(0.0))
                }
            }

            // Apply drive command
            robot.drive.joystickDrive(forward, strafe, rotation, isFieldCentric = true)

            // ── A Button: Start Slamtake Sequence ──
            val aPressed = controllerState.a
            val isSlamtakeActive = robot.store.state.superstructure.marvin.slamtakeActive
            if (aPressed && !isSlamtakeActive) {
                robot.store.dispatch(StartSlamtake())
            }

            // ── Left Bumper: Unjam ──
            val lbPressed = controllerState.leftBumper

            // ── Left Trigger: Intake/Feeder active run ──
            val ltPressed = controllerState.leftTrigger > 0.5f
            val copilotLtPressed = coPilotControllerState.leftTrigger > 0.5f

            // ── POV Left/Right: Manual Intake Deploy Override ──
            when {
                controllerState.dpadRight -> intakeDeployed = true
                controllerState.dpadLeft -> intakeDeployed = false
            }

            // Dispatch states according to pilot control priorities
            var targetPivot = intakeDeployed
            var targetIntakeRollers = 0.0
            var targetFloorSpeed = 0.0
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
                    // The reducer's active slamtake state machine owns these targets.
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
                    if (!rtPressed && !rbPressed && !bPressed) {
                        targetFloorSpeed = 0.0
                        targetFeederSpeed = 0.0
                    } else {
                        targetFloorSpeed = marvin.floor.targetVelocityRps
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
            val povUp = controllerState.dpadUp || coPilotControllerState.dpadUp
            val povDown = controllerState.dpadDown || coPilotControllerState.dpadDown
            val targetClimberVoltage = when {
                povUp -> 6.0
                povDown -> -6.0
                else -> 0.0
            }
            if (marvin.climber.targetVoltage != targetClimberVoltage) {
                robot.store.dispatch(SetClimberVoltage(targetClimberVoltage))
            }

            // ── Beach / Traction Loss detection ──
            val beached = robot.isBeached
            if (beached != lastBeached) {
                robot.telemetry.putBoolean("Diagnostics/Beached", beached)
                lastBeached = beached
                if (beached) {
                    controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
                    coPilotController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
                    rumbleStartTimestampMs = com.areslib.util.RobotClock.currentTimeMillis()
                } else {
                    controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
                    coPilotController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
                }
            } else if (beached && com.areslib.util.RobotClock.currentTimeMillis() - rumbleStartTimestampMs > 1000) {
                controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
                coPilotController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
            }
        } catch (e: Exception) {
            DriverStation.reportError("Exception in teleopPeriodic: ${e.message}", false)
            robot.safeHardware()
        }
    }
}
