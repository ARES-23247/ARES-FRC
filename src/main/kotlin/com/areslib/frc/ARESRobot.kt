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
    private lateinit var sim: Dyn4jSimulation
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
    private val RED_SPEAKER = Translation2d(11.915, 4.035)
    private val BLUE_SPEAKER = Translation2d(4.625, 4.035)

    // Simulation timing
    private var lastSimTime = 0.0


    override fun robotInit() {
        sim = Dyn4jSimulation(seed = 42L)
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
            /**
             * Documentation for can2Bus
             */
            val can2Bus = com.ctre.phoenix6.CANBus("CAN2")
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

            val ctreDrivetrain = frc.robot.generated.TunerConstants.TunerSwerveDrivetrain(
                frc.robot.generated.TunerConstants.DrivetrainConstants,
                frc.robot.generated.TunerConstants.FrontLeft,
                frc.robot.generated.TunerConstants.FrontRight,
                frc.robot.generated.TunerConstants.BackLeft,
                frc.robot.generated.TunerConstants.BackRight
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
            swerveIO = null
            visionIO = null
            flywheelIO = sim.flywheelIO
            cowlIO = sim.cowlIO
            intakeIO = sim.intakeIO
            feederIO = sim.feederIO
            floorIO = sim.floorIO
            climberIO = sim.climberIO
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
            telemetry.putNumber("Superstructure/Flywheel/VelocityRpm", marvin.flywheel.velocityRpm)
            telemetry.putNumber("Superstructure/Flywheel/TargetVelocityRpm", marvin.flywheel.targetVelocityRpm)
            telemetry.putNumber("Superstructure/Cowl/AngleRotations", marvin.cowl.angleRotations)
            telemetry.putNumber("Superstructure/Cowl/TargetAngleRotations", marvin.cowl.targetAngleRotations)
            telemetry.putNumber("Superstructure/Intake/PivotAngleDegrees", marvin.intake.pivotAngleDegrees)
            telemetry.putNumber("Superstructure/Intake/TargetAngleDegrees", marvin.intake.targetAngleDegrees)
            telemetry.putBoolean("Superstructure/Intake/IsDeployed", marvin.intake.isDeployed)
            telemetry.putNumber("Superstructure/Intake/RollerVelocityRps", marvin.intake.rollerVelocityRps)
            telemetry.putNumber("Superstructure/Feeder/VelocityRps", marvin.feeder.velocityRps)
            telemetry.putBoolean("Superstructure/Feeder/PieceDetected", marvin.feeder.gamePieceDetected)
            telemetry.putNumber("Superstructure/Floor/VelocityRps", marvin.floor.velocityRps)
            telemetry.putNumber("Superstructure/Climber/ExtensionMeters", marvin.climber.extensionMeters)
            telemetry.putNumber("Superstructure/Climber/TargetVoltage", marvin.climber.targetVoltage)
            telemetry.putBoolean("Superstructure/SlamtakeActive", marvin.slamtakeActive)

            // Log individual hardware devices via logTelemetry
            flywheelIO.logTelemetry(telemetry, "Hardware/Motors/Flywheel")
            cowlIO.logTelemetry(telemetry, "Hardware/Motors/Cowl")
            intakeIO.logTelemetry(telemetry, "Hardware/Motors/Intake")
            feederIO.logTelemetry(telemetry, "Hardware/Motors/Feeder")
            floorIO.logTelemetry(telemetry, "Hardware/Motors/Floor")
            climberIO.logTelemetry(telemetry, "Hardware/Motors/Climber")
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
        // Unified update: reads sensors, writes outputs, publishes telemetry + CSV
        robot.update(controllerState, coPilotControllerState)
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
        val actions = sim.step(robot.store.state, dt)
        for (action in actions) {
            robot.store.dispatch(action)
        }

        // Dispatch pose update so the state has odometry
        /**
         * Documentation for poseUpdate
         */
        val poseUpdate = sim.getPoseUpdate()
        robot.store.dispatch(poseUpdate)

        // Publish 3D visualization
        sim.publishVisualization(robot.store.state, robot.telemetry)
    }
}
