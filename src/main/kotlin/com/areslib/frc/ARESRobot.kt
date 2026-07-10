package com.areslib.frc

import com.areslib.action.RobotAction
import com.areslib.control.HolonomicDriveController
import com.areslib.control.PIDController
import com.areslib.control.ShotResult
import com.areslib.control.ShotSetup
import com.areslib.math.ChassisSpeeds
import com.areslib.math.Pose2d
import com.areslib.math.Rotation2d
import com.areslib.math.Translation2d
import com.areslib.pathing.Path
import com.areslib.reducer.rootReducer
import com.areslib.frc.action.*
import com.areslib.frc.subsystem.*
import com.areslib.frc.state.marvinXIX

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.DriverStation
import com.ctre.phoenix6.swerve.SwerveDrivetrain

/**
 * Main Robot lifecycle for the FRC CTRE Swerve Integration.
 *
 * This is now a thin shell that delegates all state management, hardware IO,
 * and telemetry to the [FrcSwerveRobot] facade — mirroring how FTC's
 * ARESMecanumTeleOp delegates to FtcMecanumRobot.
 */
class ARESRobot : TimedRobot() {

    private lateinit var robot: FrcSwerveRobot
    private lateinit var sim: Dyn4jSimulation
    private val controller = XboxController(0)
    private val coPilotController = XboxController(1)

    private var intakeDeployed = false
    private var driverYawOffset = 0.0

    // Pre-allocated ShotResult for SOTM (zero-allocation)
    private val shotResult = ShotResult()
    private var speakerTranslation = Translation2d(4.625, 4.035)

    // Pre-allocated objects
    private val targetPosesRed = arrayOf(Translation2d(14.6, 6.0), Translation2d(14.6, 2.0))
    private val targetPosesBlue = arrayOf(Translation2d(2.0, 6.0), Translation2d(2.0, 2.0))
    private val targetPoseScratch = DoubleArray(3)

    // Simulation timing
    private var lastSimTime = 0.0

    // Autonomous
    private var activePath: Path? = null
    private var autoStartTime = 0.0
    private var autoDistance = 0.0
    private val driveController = HolonomicDriveController(
        PIDController(4.0, 0.0, 0.1),
        PIDController(4.0, 0.0, 0.1),
        PIDController(3.0, 0.0, 0.0)
    )

    override fun robotInit() {
        sim = Dyn4jSimulation(seed = 42L)

        val isReal = RobotBase.isReal()
        robot = if (isReal) {
            try {
                FrcSwerveRobot.createPhysicalMarvinXIX()
            } catch (e: Exception) {
                println("Failed to initialize physical hardware: ${e.message}")
                // Fallback to sim IO
                FrcSwerveRobot(
                    flywheelIO = sim.flywheelIO,
                    cowlIO = sim.cowlIO,
                    intakeIO = sim.intakeIO,
                    feederIO = sim.feederIO,
                    floorIO = sim.floorIO,
                    climberIO = sim.climberIO,
                    isSimulation = true
                )
            }
        } else {
            FrcSwerveRobot(
                flywheelIO = sim.flywheelIO,
                cowlIO = sim.cowlIO,
                intakeIO = sim.intakeIO,
                feederIO = sim.feederIO,
                floorIO = sim.floorIO,
                climberIO = sim.climberIO,
                isSimulation = true
            )
        }

        lastSimTime = com.areslib.util.RobotClock.currentTimeMillis() / 1000.0

        // Wire brownout guard to read live battery voltage from roboRIO
        robot.batteryVoltageSupplier = {
            try {
                edu.wpi.first.wpilibj.RobotController.getBatteryVoltage()
            } catch (_: Exception) {
                12.6 // Fallback for simulation environments
            }
        }
    }

    override fun robotPeriodic() {
        // Unified update: reads sensors, writes outputs, publishes telemetry + CSV
        robot.update(controller.toState(), coPilotController.toState())
    }

    // ── Teleop ──

    override fun teleopInit() {
        driverYawOffset = 0.0
    }

    override fun teleopPeriodic() {
        try {
            val alliance = DriverStation.getAlliance()
            if (alliance.isPresent) {
                speakerTranslation = if (alliance.get() == DriverStation.Alliance.Red) {
                    Translation2d(11.915, 4.035)
                } else {
                    Translation2d(4.625, 4.035)
                }
            }

            val applyDeadband = { value: Double -> if (Math.abs(value) < 0.1) 0.0 else value }
            val rawForward = applyDeadband(-controller.leftY) * 4.5
            val rawStrafe = applyDeadband(-controller.leftX) * 4.5
            
            // Rotate joystick translation inputs by driverYawOffset to make controls relative to the driver's reset heading
            val cosOffset = Math.cos(driverYawOffset)
            val sinOffset = Math.sin(driverYawOffset)
            val forward = rawForward * cosOffset - rawStrafe * sinOffset
            val strafe = rawForward * sinOffset + rawStrafe * cosOffset
            
            var rotation = applyDeadband(-controller.rightX) * Math.PI

            val currentPose = Pose2d(
                robot.store.state.drive.odometryX,
                robot.store.state.drive.odometryY,
                Rotation2d(robot.store.state.drive.odometryHeading)
            )

            // ── Copilot Swerve Lock Override ──
            if (coPilotController.xButton) {
                robot.drive.joystickDrive(0.0, 0.0, 0.0)
                return
            }

            // ── Gyro Reset (Driver Coordinate Alignment) ──
            if (controller.backButton || coPilotController.backButton) {
                driverYawOffset = robot.store.state.drive.odometryHeading
            }

            // ── Driver / Copilot Shooting Triggers ──
            val rtPressed = controller.rightTriggerAxis > 0.5
            val rbPressed = controller.rightBumperButton
            val bPressed = controller.bButton
            val copilotRtPressed = coPilotController.rightTriggerAxis > 0.5
            val copilotRbPressed = coPilotController.rightBumperButton

            when {
                rtPressed -> {
                    // Shoot-on-the-Move (SOTM) Speaker Aiming
                    rotation = robot.marvinShooter.updateShootOnTheMove(
                        currentPose = currentPose,
                        targetTranslation = speakerTranslation,
                        shotResult = shotResult
                    )
                }
                rbPressed -> {
                    // Aim and Shuttle
                    val isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
                    val shuttleTarget = if (isRed) targetPosesRed[1] else targetPosesBlue[1]

                    rotation = robot.marvinShooter.updateShootOnTheMove(
                        currentPose = currentPose,
                        targetTranslation = shuttleTarget,
                        shotResult = shotResult,
                        runFloorRollers = true
                    )
                }
                bPressed -> {
                    // Static Shoot (Speaker Aiming)
                    rotation = robot.marvinShooter.updateStaticShoot(
                        currentPose = currentPose,
                        targetTranslation = speakerTranslation
                    )
                }
                copilotRtPressed -> {
                    // Copilot manual Hub shot reference
                    robot.store.dispatch(SetFlywheelSpeed(3350.0))
                    robot.store.dispatch(SetCowlAngle(0.5))
                    robot.store.dispatch(RobotAction.SetFlywheelActive(true, com.areslib.util.RobotClock.currentTimeMillis()))
                }
                copilotRbPressed -> {
                    // Copilot manual Front of Ladder shot reference
                    robot.store.dispatch(SetFlywheelSpeed(3650.0))
                    robot.store.dispatch(SetCowlAngle(1.1))
                    robot.store.dispatch(RobotAction.SetFlywheelActive(true, com.areslib.util.RobotClock.currentTimeMillis()))
                }
                else -> {
                    robot.store.dispatch(RobotAction.SetFlywheelActive(false, com.areslib.util.RobotClock.currentTimeMillis()))
                }
            }

            // Apply drive command
            robot.drive.joystickDrive(forward, strafe, rotation)

            // ── A Button: Start Slamtake Sequence ──
            val aPressed = controller.aButton
            val isSlamtakeActive = robot.store.state.superstructure.marvinXIX.slamtakeActive
            if (aPressed && !isSlamtakeActive) {
                robot.store.dispatch(StartSlamtake())
            }

            // ── Left Bumper: Unjam ──
            val lbPressed = controller.leftBumperButton

            // ── Left Trigger: Intake/Feeder active run ──
            val ltPressed = controller.leftTriggerAxis > 0.5
            val copilotLtPressed = coPilotController.leftTriggerAxis > 0.5

            // ── POV Left/Right: Manual Intake Deploy Override ──
            when (controller.pov) {
                90 -> intakeDeployed = true
                270 -> intakeDeployed = false
            }

            // Dispatch states according to pilot control priorities
            when {
                lbPressed -> {
                    // Unjam sequence takes top priority
                    if (isSlamtakeActive) {
                        robot.store.dispatch(StopSlamtake())
                    }
                    robot.store.dispatch(SetIntakePivot(deployed = true))
                    robot.store.dispatch(SetIntakeRollers(-5.0))
                    robot.store.dispatch(SetFloorSpeed(-5.0))
                    robot.store.dispatch(SetFeederSpeed(-5.0))
                }
                isSlamtakeActive -> {
                    // Handled inside MarvinReducer!
                }
                ltPressed -> {
                    // Active manual intake
                    robot.store.dispatch(SetIntakePivot(deployed = true))
                    robot.store.dispatch(SetIntakeRollers(10.0))
                    robot.store.dispatch(SetFloorSpeed(10.0))
                    robot.store.dispatch(SetFeederSpeed(10.0))
                }
                copilotLtPressed -> {
                    // Copilot manual feed override
                    robot.store.dispatch(SetIntakePivot(deployed = intakeDeployed))
                    robot.store.dispatch(SetIntakeRollers(10.0))
                    robot.store.dispatch(SetFloorSpeed(10.0))
                    robot.store.dispatch(SetFeederSpeed(10.0))
                }
                else -> {
                    // Default stop everything
                    robot.store.dispatch(SetIntakePivot(deployed = intakeDeployed))
                    robot.store.dispatch(SetIntakeRollers(0.0))
                    robot.store.dispatch(SetFloorSpeed(0.0))
                    if (!rtPressed && !rbPressed && !bPressed) {
                        robot.store.dispatch(SetFeederSpeed(0.0))
                    }
                }
            }

            // ── POV Up/Down: Climber Voltage (Driver or Copilot) ──
            val povUp = controller.pov == 0 || coPilotController.pov == 0
            val povDown = controller.pov == 180 || coPilotController.pov == 180
            when {
                povUp -> robot.store.dispatch(SetClimberVoltage(6.0))
                povDown -> robot.store.dispatch(SetClimberVoltage(-6.0))
                else -> robot.store.dispatch(SetClimberVoltage(0.0))
            }

            // ── Beach / Traction Loss detection ──
            val beached = robot.isBeached
            robot.telemetry.putBoolean("Diagnostics/Beached", beached)
            if (beached) {
                controller.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 1.0)
                coPilotController.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 1.0)
            } else {
                controller.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0)
                coPilotController.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0)
            }
        } catch (e: Throwable) {
            System.err.println("ARESRobot: Exception in teleopPeriodic: ${e.message}")
            e.printStackTrace()
            robot.safeHardware()
        }
    }

    // ── Autonomous ──

    override fun autonomousInit() {
        try {
            var path = PathLoader.loadPath("SimPath")
            val alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            val aresAlliance = if (alliance == DriverStation.Alliance.Red) com.areslib.state.Alliance.RED else com.areslib.state.Alliance.BLUE
            
            path = com.areslib.math.AllianceMirroring.mirror(
                path,
                aresAlliance,
                com.areslib.math.FieldSymmetry.MIRRORED,
                fieldLength = com.areslib.math.CoordinateTransformers.FRC_FIELD_LENGTH,
                fieldWidth = com.areslib.math.CoordinateTransformers.FRC_FIELD_WIDTH
            )
            activePath = path

            val startPoint = activePath?.points?.firstOrNull()
            if (startPoint != null) {
                sim.resetPose(startPoint.pose.x, startPoint.pose.y, startPoint.pose.heading.radians)
                robot.store.dispatch(RobotAction.PoseUpdate(
                    xMeters = startPoint.pose.x,
                    yMeters = startPoint.pose.y,
                    headingRadians = startPoint.pose.heading.radians,
                    timestampMs = com.areslib.util.RobotClock.currentTimeMillis(),
                    isReset = true
                ))
            }
        } catch (e: Exception) {
            println("ERROR: Failed to load autonomous path SimPath: ${e.message}")
            activePath = null
        }
        autoStartTime = com.areslib.util.RobotClock.currentTimeMillis() / 1000.0
        autoDistance = 0.0
    }

    override fun autonomousPeriodic() {
        try {
            val alliance = DriverStation.getAlliance()
            if (alliance.isPresent) {
                speakerTranslation = if (alliance.get() == DriverStation.Alliance.Red) {
                    Translation2d(11.915, 4.035)
                } else {
                    Translation2d(4.625, 4.035)
                }
            }

            val path = activePath ?: return
            val dt = 0.02

            val currentPose = Pose2d(
                robot.store.state.drive.odometryX,
                robot.store.state.drive.odometryY,
                Rotation2d(robot.store.state.drive.odometryHeading)
            )

            val targetPoint = path.sampleAtDistance(autoDistance)

            val speeds = driveController.calculate(
                currentPose = currentPose,
                targetPose = targetPoint.pose,
                targetVelocityMps = targetPoint.velocityMps,
                targetHeading = targetPoint.pose.heading,
                dtSeconds = dt
            )

            // Field-relative conversion
            val cos = currentPose.heading.cos
            val sin = currentPose.heading.sin
            val fieldX = speeds.vxMetersPerSecond * cos - speeds.vyMetersPerSecond * sin
            val fieldY = speeds.vxMetersPerSecond * sin + speeds.vyMetersPerSecond * cos

            robot.drive.joystickDrive(fieldX, fieldY, speeds.omegaRadiansPerSecond)

            // Event markers
            for (event in path.events) {
                val nextDistance = autoDistance + targetPoint.velocityMps * dt
                if (event.triggerDistanceMeters in autoDistance..nextDistance) {
                    println("AUTO EVENT TRIGGERED: ${event.eventName} at ${event.triggerDistanceMeters}m")
                    robot.telemetry.putString("Robot/ActiveEvent", event.eventName)
                    when (event.eventName) {
                        "FlywheelOn" -> robot.marvinShooter.spinUp(4000.0)
                        "IntakeDeploy" -> {
                            robot.marvinIntake.deploy()
                            robot.marvinIntake.setRollerSpeed(15.0)
                        }
                        "FeederShoot" -> robot.marvinShooter.shoot()
                    }
                }
            }

            // Trajectory telemetry
            targetPoseScratch[0] = targetPoint.pose.x
            targetPoseScratch[1] = targetPoint.pose.y
            targetPoseScratch[2] = targetPoint.pose.heading.radians
            robot.telemetry.putDoubleArray("Robot/TargetPose", targetPoseScratch)
            val dx = targetPoint.pose.x - currentPose.x
            val dy = targetPoint.pose.y - currentPose.y
            robot.telemetry.putNumber("Robot/TrajectoryError", kotlin.math.hypot(dx, dy))

            autoDistance += targetPoint.velocityMps * dt
        } catch (e: Throwable) {
            System.err.println("ARESRobot: Exception in autonomousPeriodic: ${e.message}")
            e.printStackTrace()
            robot.safeHardware()
        }
    }

    // ── Simulation ──

    override fun simulationPeriodic() {
        if (!RobotBase.isSimulation()) return

        val now = com.areslib.util.RobotClock.currentTimeMillis() / 1000.0
        val dt = Math.min(now - lastSimTime, 0.05)
        lastSimTime = now

        // Step physics and dispatch any resulting actions (ball intake/shoot)
        val actions = sim.step(robot.store.state, dt)
        for (action in actions) {
            robot.store.dispatch(action)
        }

        // Feed sim pose back into Store
        robot.store.dispatch(sim.getPoseUpdate())

        // Publish 3D visualization
        sim.publishVisualization(robot.store.state, robot.telemetry)
    }
}
