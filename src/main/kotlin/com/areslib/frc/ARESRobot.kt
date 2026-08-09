package com.areslib.frc

import com.areslib.action.RobotAction
import com.areslib.control.drivetrain.HolonomicDriveController
import com.areslib.control.feedback.PIDController
import com.areslib.control.assist.ShotResult
import com.areslib.control.assist.ShotSetup
import com.areslib.math.geometry.ChassisSpeeds
import com.areslib.math.geometry.Pose2d
import com.areslib.math.geometry.Rotation2d
import com.areslib.math.geometry.Translation2d
import com.areslib.pathing.Path
import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import com.areslib.state.VisionState
import com.areslib.reducer.rootReducer
import com.areslib.frc.marvin.*
import com.areslib.telemetry.GamepadState
import com.areslib.frc.hardware.FlywheelIO
import com.areslib.frc.hardware.CowlIO
import com.areslib.frc.hardware.IntakeIO
import com.areslib.frc.hardware.FeederIO
import com.areslib.frc.hardware.FloorIO
import com.areslib.frc.hardware.ClimberIO
import com.areslib.frc.hardware.FRCClimberHardwareIO
import com.areslib.frc.hardware.FRCCowlHardwareIO
import com.areslib.frc.hardware.FRCFeederHardwareIO
import com.areslib.frc.hardware.FRCFloorHardwareIO
import com.areslib.frc.hardware.FRCFlywheelHardwareIO
import com.areslib.frc.hardware.FRCIntakeHardwareIO

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.VecBuilder

import com.areslib.hardware.SubsystemIO
import com.areslib.hardware.drive.SwerveHardwareIO
import com.areslib.hardware.vision.VisionIO

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.DriverStation

import com.areslib.frc.robot.FRCAutoOrchestrator
import com.areslib.frc.robot.FRCTeleOpDriveController
/**
 * Documentation for aresAlliance
 */

val aresAlliance: com.areslib.state.Alliance
    get() = if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
        com.areslib.state.Alliance.RED
    } else {
        com.areslib.state.Alliance.BLUE
    }

/**
 * Main Robot lifecycle for the FRC CTRE Swerve Integration.
 *
 * This is now a thin shell that delegates all state management, hardware IO,
 * and telemetry to the [FrcSwerveRobot] facade — mirroring how FTC's
 * ARESMecanumTeleOp delegates to FtcMecanumRobot.
 */
class ARESRobot : TimedRobot() {

    private lateinit var robot: FrcSwerveRobot
    private var sim: Dyn4jSimulation? = null
    private lateinit var marvinShooter: MarvinShooterSubsystem
    private lateinit var marvinIntake: MarvinIntakeSubsystem
    private lateinit var marvinClimber: MarvinClimberSubsystem

    private val controller = XboxController(0)
    private val coPilotController = XboxController(1)
    private val controllerState = GamepadState()
    private val coPilotControllerState = GamepadState()

    private lateinit var teleOpController: FRCTeleOpDriveController
    private lateinit var autoOrchestrator: FRCAutoOrchestrator

    private var cachedAlliance: DriverStation.Alliance = DriverStation.Alliance.Blue
    private val RED_SPEAKER = MarvinConfig.FieldTargets.redSpeaker
    private val BLUE_SPEAKER = MarvinConfig.FieldTargets.blueSpeaker

    // Simulation timing
    private var lastSimTime = 0.0
    private val can2Bus = com.ctre.phoenix6.CANBus("CAN2")


    override fun robotInit() {
        edu.wpi.first.wpilibj.Threads.setCurrentThreadPriority(true, 10)
        /**
         * Documentation for isReal
         */

        val isReal = RobotBase.isReal()

        // 1. Declare the hardware IO instances (either physical or simulation)
        /**
         * Documentation for swerveIO
         */
        val swerveIO: SwerveHardwareIO?
        /**
         * Documentation for visionIO
         */
        val visionIO: VisionIO?
        /**
         * Documentation for flywheelIO
         */
        val flywheelIO: FlywheelIO
        /**
         * Documentation for cowlIO
         */
        val cowlIO: CowlIO
        /**
         * Documentation for intakeIO
         */
        val intakeIO: IntakeIO
        /**
         * Documentation for feederIO
         */
        val feederIO: FeederIO
        /**
         * Documentation for floorIO
         */
        val floorIO: FloorIO
        /**
         * Documentation for climberIO
         */
        val climberIO: ClimberIO

        if (isReal) {
            // can2Bus is already defined as a class property
            /**
             * Documentation for leftMasterFX
             */
            val leftMasterFX = com.ctre.phoenix6.hardware.TalonFX(9, can2Bus)
            /**
             * Documentation for leftFollowerFX
             */
            val leftFollowerFX = com.ctre.phoenix6.hardware.TalonFX(10, can2Bus)
            /**
             * Documentation for rightMasterFX
             */
            val rightMasterFX = com.ctre.phoenix6.hardware.TalonFX(11, can2Bus)
            /**
             * Documentation for rightFollowerFX
             */
            val rightFollowerFX = com.ctre.phoenix6.hardware.TalonFX(12, can2Bus)
            /**
             * Documentation for cowlFX
             */
            val cowlFX = com.ctre.phoenix6.hardware.TalonFX(13, can2Bus)
            /**
             * Documentation for pivotFX
             */
            val pivotFX = com.ctre.phoenix6.hardware.TalonFX(14, can2Bus)
            /**
             * Documentation for rollerFX
             */
            val rollerFX = com.ctre.phoenix6.hardware.TalonFX(15, can2Bus)
            /**
             * Documentation for floorFX
             */
            val floorFX = com.ctre.phoenix6.hardware.TalonFX(16, can2Bus)
            /**
             * Documentation for climberFX
             */
            val climberFX = com.ctre.phoenix6.hardware.TalonFX(19, can2Bus)
            /**
             * Documentation for feederFX
             */
            val feederFX = com.ctre.phoenix6.hardware.TalonFX(20, can2Bus)
            /**
             * Documentation for ctreDrivetrain
             */

            val defaultOffsets = frc.robot.generated.TunerConstants.getDefaultOffsets()
            val activeOffsets = com.areslib.drivetrain.SwerveOffsetManager.loadOffsets(defaultOffsets)

            val ctreDrivetrain = frc.robot.generated.TunerConstants.TunerSwerveDrivetrain(
                frc.robot.generated.TunerConstants.DrivetrainConstants,
                0.0,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.9, 0.9, 0.9),
                frc.robot.generated.TunerConstants.createFrontLeft(edu.wpi.first.units.Units.Rotations.of(activeOffsets.frontLeft)),
                frc.robot.generated.TunerConstants.createFrontRight(edu.wpi.first.units.Units.Rotations.of(activeOffsets.frontRight)),
                frc.robot.generated.TunerConstants.createBackLeft(edu.wpi.first.units.Units.Rotations.of(activeOffsets.backLeft)),
                frc.robot.generated.TunerConstants.createBackRight(edu.wpi.first.units.Units.Rotations.of(activeOffsets.backRight))
            )
            swerveIO = FRCSwerveHardwareIO(ctreDrivetrain)
            /**
             * Documentation for limelightShooter
             */

            val limelightShooter = FrcLimelightIO("limelight-shooter")
            /**
             * Documentation for limelightBack
             */
            val limelightBack = FrcLimelightIO("limelight-back")
            visionIO = com.areslib.hardware.vision.CompositeVisionIO(listOf(limelightShooter, limelightBack))

            flywheelIO = FRCFlywheelHardwareIO(leftMasterFX, leftFollowerFX, rightMasterFX, rightFollowerFX)
            cowlIO = FRCCowlHardwareIO(cowlFX)
            intakeIO = FRCIntakeHardwareIO(pivotFX, rollerFX)
            feederIO = FRCFeederHardwareIO(feederFX)
            floorIO = FRCFloorHardwareIO(floorFX)
            climberIO = FRCClimberHardwareIO(climberFX)
        } else {
            // Simulation IOs
            val simInstance = Dyn4jSimulation(seed = 42L)
            sim = simInstance
            swerveIO = null
            visionIO = null
            flywheelIO = simInstance.flywheelIO
            cowlIO = simInstance.cowlIO
            intakeIO = simInstance.intakeIO
            feederIO = simInstance.feederIO
            floorIO = simInstance.floorIO
            climberIO = simInstance.climberIO
        }

        // Register subsystems to HardwareRegistry so they are refreshed/logged automatically
        flywheelIO?.let { com.areslib.hardware.HardwareRegistry.registerDevice("Flywheel", it) }
        cowlIO?.let { com.areslib.hardware.HardwareRegistry.registerDevice("Cowl", it) }
        intakeIO?.let { com.areslib.hardware.HardwareRegistry.registerDevice("Intake", it) }
        feederIO?.let { com.areslib.hardware.HardwareRegistry.registerDevice("Feeder", it) }
        floorIO?.let { com.areslib.hardware.HardwareRegistry.registerDevice("Floor", it) }
        climberIO?.let { com.areslib.hardware.HardwareRegistry.registerDevice("Climber", it) }

        // 2. Compose the root reducer with the Marvin reducer
        /**
         * Documentation for composedReducer
         */
        fun composedReducer(state: RobotState, action: RobotAction): RobotState {
            return MarvinReducer.reduce(state, action)
        }

        // 3. Create the initial state containing the MarvinState
        /**
         * Documentation for initialState
         */
        val initialState = RobotState(
            superstructure = SuperstructureState(
                custom = MarvinState()
            ),
            vision = VisionState(
                filterConfig = com.areslib.hardware.vision.VisionFilterConfig.frcDefaults()
            )
        )

        // 4. Instantiate FrcSwerveRobot
        robot = FrcSwerveRobot(
            swerveIO = swerveIO,
            visionIO = visionIO,
            isSimulation = !isReal,
            initialState = initialState,
            reducer = ::composedReducer
        )

        robot.store.actionListener = { action ->
            if (action is RobotAction.CalibrateSwerveOffsets && swerveIO != null) {
                val encPositions = DoubleArray(4)
                swerveIO?.getEncoderPositions(encPositions)
                val defaultOffsets = frc.robot.generated.TunerConstants.getDefaultOffsets()
                val activeOffsets = com.areslib.drivetrain.SwerveOffsetManager.loadOffsets(defaultOffsets)
                val newOffsets = com.areslib.drivetrain.SwerveOffsetData(
                    frontLeft = activeOffsets.frontLeft - encPositions[0],
                    frontRight = activeOffsets.frontRight - encPositions[1],
                    backLeft = activeOffsets.backLeft - encPositions[2],
                    backRight = activeOffsets.backRight - encPositions[3]
                )
                com.areslib.drivetrain.SwerveOffsetManager.saveRuntimeOffsets(
                    newOffsets,
                    robot.telemetryManager.dataLoggingTelemetry
                )
            }
        }

        // 5. Create and register the MarvinSuperstructure subsystem
        /**
         * Documentation for superstructureSubsystem
         */
        val superstructureSubsystem = MarvinSuperstructure(
            flywheelIO = flywheelIO,
            cowlIO = cowlIO,
            intakeIO = intakeIO,
            feederIO = feederIO,
            floorIO = floorIO,
            climberIO = climberIO
        )
        robot.registerSubsystem(superstructureSubsystem)

        // 6. Instantiate the facades
        marvinShooter = MarvinShooterSubsystem(robot.store)
        marvinIntake = MarvinIntakeSubsystem(robot.store)
        marvinClimber = MarvinClimberSubsystem(robot.store)

        // 7. Register a custom telemetry publisher for Marvin state
        robot.telemetryManager.customPublishers.add { state, telemetry ->
            /**
             * Documentation for marvin
             */
            val marvin = state.superstructure.marvin
            // Log Marvin state
            val telemetryArray = doubleArrayOf(
                marvin.flywheel.velocityRpm,
                marvin.flywheel.targetVelocityRpm,
                marvin.cowl.angleRotations,
                marvin.cowl.targetAngleRotations,
                marvin.intake.pivotAngleDegrees,
                marvin.intake.targetAngleDegrees,
                if (marvin.intake.isDeployed) 1.0 else 0.0,
                marvin.intake.rollerVelocityRps,
                marvin.feeder.velocityRps,
                if (marvin.feeder.gamePieceDetected) 1.0 else 0.0,
                marvin.floor.velocityRps,
                marvin.climber.extensionMeters,
                marvin.climber.targetVoltage,
                if (marvin.slamtakeActive) 1.0 else 0.0
            )
            telemetry.putDoubleArray("Superstructure/PackedState", telemetryArray)
            if (edu.wpi.first.wpilibj.RobotBase.isReal()) {
                val loopCounter = (state.timestampMs / 20) // 50Hz
                if (loopCounter % 25L == 0L) { // 2Hz
                    telemetry.putNumber("CAN2/BusUtilization", can2Bus.status.BusUtilization.toDouble())
                }
            }
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

        teleOpController = FRCTeleOpDriveController(
            robot, marvinShooter, marvinIntake, marvinClimber,
            controller, coPilotController, controllerState, coPilotControllerState
        )
        autoOrchestrator = FRCAutoOrchestrator(
            robot, sim, marvinShooter, marvinIntake
        )

        addPeriodic({
            try {
                robot.update(controllerState, coPilotControllerState)
            } catch (e: Exception) {
                DriverStation.reportError("Periodic loop exception", false)
                robot.safeHardware()
            }
        }, 0.02, 0.005)
    }

    private var allianceCheckCounter = 0

    override fun robotPeriodic() {
        if (DriverStation.isDisabled() && allianceCheckCounter++ % 50 == 0) {
            /**
             * Documentation for allianceOpt
             */
            val allianceOpt = DriverStation.getAlliance()
            if (allianceOpt.isPresent) {
                /**
                 * Documentation for alliance
                 */
                val alliance = allianceOpt.get()
                if (alliance != cachedAlliance) {
                    cachedAlliance = alliance
                    teleOpController.cachedAlliance = alliance
                    teleOpController.speakerTranslation = if (alliance == DriverStation.Alliance.Red) RED_SPEAKER else BLUE_SPEAKER
                }
            }
        }
        controller.updateState(controllerState)
        coPilotController.updateState(coPilotControllerState)
    }

    override fun disabledInit() {
        controller.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0)
        coPilotController.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0)
    }

    override fun disabledPeriodic() {
    }

    // ── Teleop ──

    override fun teleopInit() {
        teleOpController.teleopInit()
    }

    override fun teleopPeriodic() {
        teleOpController.teleopPeriodic()
    }

    // ── Autonomous ──

    override fun autonomousInit() {
        autoOrchestrator.autonomousInit()
    }

    override fun autonomousPeriodic() {
        autoOrchestrator.autonomousPeriodic()
    }

    // ── Simulation ──

    override fun simulationPeriodic() {
        if (!RobotBase.isSimulation()) return
        val simInstance = sim ?: return
        /**
         * Documentation for now
         */

        val now = com.areslib.util.RobotClock.currentTimeMillis() / 1000.0
        /**
         * Documentation for dt
         */
        val dt = Math.min(now - lastSimTime, 0.05)
        lastSimTime = now

        // Step physics and dispatch any resulting actions (ball intake/shoot)
        /**
         * Documentation for actions
         */
        val actions = simInstance.step(robot.store.state, dt)
        for (action in actions) {
            robot.store.dispatch(action)
        }

        // Dispatch pose update so the state has odometry
        /**
         * Documentation for poseUpdate
         */
        val poseUpdate = simInstance.getPoseUpdate()
        robot.store.dispatch(poseUpdate)

        // Publish 3D visualization
        simInstance.publishVisualization(robot.store.state, robot.telemetry)
    }
}
