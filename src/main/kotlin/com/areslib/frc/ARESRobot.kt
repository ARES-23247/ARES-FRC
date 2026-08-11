package com.areslib.frc

import com.areslib.action.RobotAction
import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import com.areslib.state.VisionState
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

import edu.wpi.first.math.VecBuilder

import com.areslib.hardware.drive.SwerveHardwareIO
import com.areslib.hardware.vision.VisionIO

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.DriverStation

import com.areslib.frc.robot.FRCAutoOrchestrator
import com.areslib.frc.robot.FrcAutoCapabilities
import com.areslib.frc.robot.FRCTeleOpDriveController
import com.areslib.frc.robot.FrcSysIdController
import com.areslib.frc.vision.FrcLocalizationCalibrationSession
import com.areslib.frc.vision.FrcVisionTracker
import com.areslib.frc.generatedruntime.FrcControllerBindingHost
import com.areslib.frc.generated.GeneratedAresProject
import com.areslib.frc.generated.subsystems.GeneratedSubsystemRegistry
import com.areslib.frc.generatedruntime.selectDefaultGeneratedControlScheme

/**
 * Current Driver Station alliance in the platform-neutral ARES state model.
 * Unknown station data deliberately falls back to Blue so blue-origin field transforms remain
 * defined before the Driver Station supplies an alliance.
 */
val aresAlliance: com.areslib.state.Alliance
    get() = when (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
        DriverStation.Alliance.Red -> com.areslib.state.Alliance.RED
        DriverStation.Alliance.Blue -> com.areslib.state.Alliance.BLUE
        else -> com.areslib.state.Alliance.BLUE
    }

/** Returns false when any real mechanism adapter reports failed one-time configuration. */
internal fun mechanismsConfigured(vararg devices: Any): Boolean {
    for (device in devices) {
        val status = device as? com.areslib.frc.hardware.FrcMechanismConfigurationStatus ?: continue
        if (!status.configurationValid) return false
    }
    return true
}

/**
 * WPILib lifecycle and dependency-composition root for Marvin XIX.
 *
 * This shell selects real or simulated IO, owns controller snapshots, and delegates
 * state/control work to [FrcSwerveRobot], [FRCTeleOpDriveController], and
 * [FRCAutoOrchestrator]. Its registered 20 ms ARES update refreshes hardware before
 * sensor reads and Redux-derived output writes. Mode callbacks remain orchestration-only.
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
    private lateinit var sysIdController: FrcSysIdController
    private var generatedControllerBindings: FrcControllerBindingHost? = null
    private var localizationCalibration: FrcLocalizationCalibrationSession? = null
    private var localizationVisionTracker: FrcVisionTracker? = null
    private val calibrationButtonState = BooleanArray(12)

    private var cachedAlliance: DriverStation.Alliance = DriverStation.Alliance.Blue
    private val RED_SPEAKER = MarvinConfig.FieldTargets.redSpeaker
    private val BLUE_SPEAKER = MarvinConfig.FieldTargets.blueSpeaker
    private val superstructureTelemetry = DoubleArray(14)
    private val swerveCalibrationSamples = SwerveOffsetCalibrationSampleCache()
    private val calibrationEncoderPositions = DoubleArray(SwerveOffsetCalibrationSampleCache.MODULE_COUNT)
    private var mechanismConfigurationValid = true

    // Simulation timing
    private var lastSimTime = 0.0
    private val can2Bus = com.ctre.phoenix6.CANBus("CAN2")
    private var powerDistribution: edu.wpi.first.wpilibj.PowerDistribution? = null


    /** Constructs IO, the composed reducer/store, subsystem lifecycle, and mode controllers. */
    override fun robotInit() {
        edu.wpi.first.wpilibj.Threads.setCurrentThreadPriority(true, 10)

        val isReal = RobotBase.isReal()

        // 1. Declare the hardware IO instances (either physical or simulation)
        val swerveIO: SwerveHardwareIO?
        val visionIO: VisionIO?
        val flywheelIO: FlywheelIO
        val cowlIO: CowlIO
        val intakeIO: IntakeIO
        val feederIO: FeederIO
        val floorIO: FloorIO
        val climberIO: ClimberIO

        if (isReal) {
            powerDistribution = try {
                edu.wpi.first.wpilibj.PowerDistribution()
            } catch (error: Exception) {
                DriverStation.reportError(
                    "ARES: PowerDistribution initialization failed; using cached motor-current fallback: " +
                        error.message,
                    false
                )
                null
            }
            // can2Bus is already defined as a class property
            val leftMasterFX = com.ctre.phoenix6.hardware.TalonFX(9, can2Bus)
            val leftFollowerFX = com.ctre.phoenix6.hardware.TalonFX(10, can2Bus)
            val rightMasterFX = com.ctre.phoenix6.hardware.TalonFX(11, can2Bus)
            val rightFollowerFX = com.ctre.phoenix6.hardware.TalonFX(12, can2Bus)
            val cowlFX = com.ctre.phoenix6.hardware.TalonFX(13, can2Bus)
            val pivotFX = com.ctre.phoenix6.hardware.TalonFX(14, can2Bus)
            val rollerFX = com.ctre.phoenix6.hardware.TalonFX(15, can2Bus)
            val floorFX = com.ctre.phoenix6.hardware.TalonFX(16, can2Bus)
            val climberFX = com.ctre.phoenix6.hardware.TalonFX(19, can2Bus)
            val feederFX = com.ctre.phoenix6.hardware.TalonFX(20, can2Bus)

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

            // Each Limelight retains its independently surveyed robot-space transform from
            // its own web UI. Passing no pose is intentional: never overwrite either camera
            // with a shared placeholder extrinsic.
            val crescendoTagIds = IntArray(16) { it + 1 }
            val limelightShooter = FrcLimelightIO(
                tableName = "limelight-shooter",
                validFiducialIds = crescendoTagIds
            )
            val limelightBack = FrcLimelightIO(
                tableName = "limelight-back",
                validFiducialIds = crescendoTagIds
            )
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

        mechanismConfigurationValid = mechanismsConfigured(
            flywheelIO, cowlIO, intakeIO, feederIO, floorIO, climberIO
        )

        // Register subsystems to HardwareRegistry so they are refreshed/logged automatically
        com.areslib.hardware.HardwareRegistry.registerDevice("Flywheel", flywheelIO)
        com.areslib.hardware.HardwareRegistry.registerDevice("Cowl", cowlIO)
        com.areslib.hardware.HardwareRegistry.registerDevice("Intake", intakeIO)
        com.areslib.hardware.HardwareRegistry.registerDevice("Feeder", feederIO)
        com.areslib.hardware.HardwareRegistry.registerDevice("Floor", floorIO)
        com.areslib.hardware.HardwareRegistry.registerDevice("Climber", climberIO)

        // 2. Compose the root reducer with the Marvin reducer
        fun composedReducer(state: RobotState, action: RobotAction): RobotState {
            return MarvinReducer.reduce(state, action)
        }

        // 3. Create the initial state containing the MarvinState
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
                val nowMs = com.areslib.util.RobotClock.currentTimeMillis()
                if (swerveCalibrationSamples.copyFresh(nowMs, calibrationEncoderPositions)) {
                    val defaultOffsets = frc.robot.generated.TunerConstants.getDefaultOffsets()
                    val activeOffsets = com.areslib.drivetrain.SwerveOffsetManager.loadOffsets(defaultOffsets)
                    val newOffsets = com.areslib.drivetrain.SwerveOffsetData(
                        frontLeft = activeOffsets.frontLeft - calibrationEncoderPositions[0],
                        frontRight = activeOffsets.frontRight - calibrationEncoderPositions[1],
                        backLeft = activeOffsets.backLeft - calibrationEncoderPositions[2],
                        backRight = activeOffsets.backRight - calibrationEncoderPositions[3]
                    )
                    com.areslib.drivetrain.SwerveOffsetManager.saveRuntimeOffsets(
                        newOffsets,
                        robot.telemetryManager.dataLoggingTelemetry
                    )
                } else {
                    val message = "Swerve offset calibration rejected: four fresh, finite, plausible " +
                        "absolute-encoder readings are required"
                    robot.telemetry.putString("Calibration/Swerve/Error", message)
                    DriverStation.reportError(message, false)
                }
            }
        }

        applyMechanismSafetyPolicy("initialization")

        // Generated subsystem DSL participates in the same lifecycle as handwritten mechanisms.
        GeneratedSubsystemRegistry.createAll(isReal).forEach(robot::registerSubsystem)

        // 5. Create and register the MarvinSuperstructure subsystem
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
            val marvin = state.superstructure.marvin
            // Log Marvin state
            superstructureTelemetry[0] = marvin.flywheel.velocityRpm
            superstructureTelemetry[1] = marvin.flywheel.targetVelocityRpm
            superstructureTelemetry[2] = marvin.cowl.angleRotations
            superstructureTelemetry[3] = marvin.cowl.targetAngleRotations
            superstructureTelemetry[4] = marvin.intake.pivotAngleDegrees
            superstructureTelemetry[5] = marvin.intake.targetAngleDegrees
            superstructureTelemetry[6] = if (marvin.intake.isDeployed) 1.0 else 0.0
            superstructureTelemetry[7] = marvin.intake.rollerVelocityRps
            superstructureTelemetry[8] = marvin.feeder.velocityRps
            superstructureTelemetry[9] = if (marvin.feeder.gamePieceDetected) 1.0 else 0.0
            superstructureTelemetry[10] = marvin.floor.velocityRps
            superstructureTelemetry[11] = marvin.climber.positionRotations
            superstructureTelemetry[12] = marvin.climber.targetVoltage
            superstructureTelemetry[13] = if (marvin.slamtakeActive) 1.0 else 0.0
            telemetry.putDoubleArray("Superstructure/PackedState", superstructureTelemetry)
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
                0.0 // Unknown voltage must fail closed; simulation supplies a valid value.
            }
        }
        if (isReal) {
            robot.totalCurrentSupplier = {
                powerDistribution?.totalCurrent ?: Double.NaN
            }
            robot.brownedOutSupplier = {
                edu.wpi.first.wpilibj.RobotController.isBrownedOut()
            }
        }

        teleOpController = FRCTeleOpDriveController(
            robot, marvinShooter, marvinIntake, marvinClimber,
            controller, coPilotController, controllerState, coPilotControllerState
        )
        sysIdController = FrcSysIdController(robot.telemetryManager.dataLoggingTelemetry, flywheelIO)
        applyAlliance(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue))
        FrcAutoCapabilities.register()
        autoOrchestrator = FRCAutoOrchestrator(robot, sim)
        autoOrchestrator.publishCatalog()
        installGeneratedControllerBindingsFromProject()

        addPeriodic({
            try {
                robot.update(controllerState, coPilotControllerState)
                if (swerveIO != null) {
                    swerveCalibrationSamples.record(
                        swerveIO,
                        com.areslib.util.RobotClock.currentTimeMillis()
                    )
                }
                val tuningEnabled = DriverStation.isTest() || RobotBase.isSimulation()
                robot.isLiveTuningEnabled = tuningEnabled
                sysIdController.update(com.areslib.util.RobotClock.currentTimeMillis(), robot.store.state, tuningEnabled)
            } catch (e: Exception) {
                DriverStation.reportError("Periodic loop exception", false)
                latchMechanismAllStop()
            }
        }, 0.02, 0.005)
    }

    private var allianceCheckCounter = 0

    /** Refreshes cached controller snapshots and polls alliance changes while disabled. */
    override fun robotPeriodic() {
        if (DriverStation.isDisabled() && allianceCheckCounter++ % 50 == 0) {
            val allianceOpt = DriverStation.getAlliance()
            if (allianceOpt.isPresent) {
                val alliance = allianceOpt.get()
                if (alliance != cachedAlliance) applyAlliance(alliance)
            }
        }
        controller.updateState(controllerState)
        coPilotController.updateState(coPilotControllerState)
    }

    private fun applyAlliance(alliance: DriverStation.Alliance) {
        cachedAlliance = alliance
        robot.store.dispatch(
            RobotAction.SetAlliance(
                if (alliance == DriverStation.Alliance.Red) {
                    com.areslib.state.Alliance.RED
                } else {
                    com.areslib.state.Alliance.BLUE
                }
            )
        )
        teleOpController.cachedAlliance = alliance
        teleOpController.speakerTranslation = if (alliance == DriverStation.Alliance.Red) RED_SPEAKER else BLUE_SPEAKER
    }

    override fun disabledInit() {
        if (::autoOrchestrator.isInitialized) autoOrchestrator.stop()
        if (::sysIdController.isInitialized) sysIdController.stop()
        generatedControllerBindings?.cancel()
        controller.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0)
        coPilotController.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0)
    }

    override fun disabledPeriodic() {
    }

    // ── Teleop ──

    override fun teleopInit() {
        autoOrchestrator.stop()
        teleOpController.teleopInit()
        applyMechanismSafetyPolicy("teleop initialization")
        generatedControllerBindings?.cancel()
    }

    override fun teleopPeriodic() {
        try {
            val generatedBindings = generatedControllerBindings
            if (generatedBindings == null) {
                teleOpController.teleopPeriodic()
            } else {
                generatedBindings.update()
                // Generated bindings own mechanisms. Legacy stick drive remains the explicit drive
                // owner unless a generated routine is actively following a path.
                if (!autoOrchestrator.generatedRoutineOwnsDrive()) {
                    teleOpController.drivePeriodic()
                }
            }
        } catch (error: Exception) {
            DriverStation.reportError("Teleop controller exception: ${error.message}", false)
            latchMechanismAllStop()
        }
    }

    private fun installGeneratedControllerBindingsFromProject() {
        val schemeId = selectDefaultGeneratedControlScheme(GeneratedAresProject.knownControlSchemeIds)
        if (schemeId == null) {
            robot.telemetry.putString("ARES/Controls/Source", "legacy")
            return
        }
        runCatching { autoOrchestrator.createControllerBindingHost(schemeId) }
            .onSuccess { host ->
                installGeneratedControllerBindings(host)
                robot.telemetry.putString("ARES/Controls/Source", "generated:$schemeId")
            }
            .onFailure { error ->
                installGeneratedControllerBindings(null)
                val message = "Generated controls '$schemeId' were rejected; using legacy: " +
                    (error.message ?: error::class.java.simpleName)
                robot.telemetry.putString("ARES/Controls/Error", message)
                DriverStation.reportWarning(message, false)
            }
    }

    /**
     * Installs a complete generated scheme at the platform boundary.
     *
     * The generated catalog currently declares no schemes, so production retains the proven
     * hardcoded controller. Once code generation emits a complete graph, robot initialization can
     * install it here without mixing duplicate actions from two teleop owners.
     */
    internal fun installGeneratedControllerBindings(host: FrcControllerBindingHost?) {
        generatedControllerBindings?.cancel()
        generatedControllerBindings = host
    }

    // ── Autonomous ──

    override fun autonomousInit() {
        generatedControllerBindings?.cancel()
        applyMechanismSafetyPolicy("autonomous initialization")
        applyAlliance(DriverStation.getAlliance().orElse(cachedAlliance))
        autoOrchestrator.autonomousInit()
    }

    override fun autonomousPeriodic() {
        autoOrchestrator.autonomousPeriodic()
    }

    override fun autonomousExit() {
        autoOrchestrator.stop()
    }

    // ── Localization calibration (Driver Station Test mode) ──

    override fun testInit() {
        generatedControllerBindings?.cancel()
        autoOrchestrator.stop()
        applyMechanismSafetyPolicy("test initialization")
        robot.swerveDrive.brake()
        localizationCalibration?.close()
        calibrationButtonState.fill(false)
        val tracker = robot.visionTracker as? FrcVisionTracker
        localizationVisionTracker = tracker
        localizationCalibration = FrcLocalizationCalibrationSession(
            store = robot.store,
            swerveIO = robot.swerveDrivetrainIO,
            measurementsProvider = { tracker?.visionInputs?.measurements ?: emptyList() }
        )
    }

    override fun testPeriodic() {
        val calibration = localizationCalibration ?: return
        teleOpController.drivePeriodic()
        val timestampMs = com.areslib.util.RobotClock.currentTimeMillis()
        val pov = controller.pov

        if (calibrationRising(0, controller.aButton)) calibration.toggleContinuousRecording()
        if (calibrationRising(1, controller.bButton)) calibration.cycleTestType()
        localizationVisionTracker?.fusionEnabled = when (calibration.testType) {
            com.areslib.math.estimation.LocalizationCalibrationTestType.ODOMETRY_TRANSLATION,
            com.areslib.math.estimation.LocalizationCalibrationTestType.ODOMETRY_ROTATION -> false
            else -> true
        }
        if (calibrationRising(2, controller.xButton)) calibration.markStart(timestampMs)
        if (calibrationRising(3, controller.yButton)) calibration.markEnd(timestampMs)
        if (calibrationRising(4, controller.backButton)) calibration.zeroTruth()
        if (calibrationRising(5, controller.startButton)) calibration.seedPoseToTruth(timestampMs)
        if (calibrationRising(6, controller.leftBumperButton)) {
            calibration.adjustTruth(deltaHeading = -Math.toRadians(5.0))
        }
        if (calibrationRising(7, controller.rightBumperButton)) {
            calibration.adjustTruth(deltaHeading = Math.toRadians(5.0))
        }
        if (calibrationRising(8, pov == 0)) calibration.adjustTruth(deltaY = 0.05)
        if (calibrationRising(9, pov == 180)) calibration.adjustTruth(deltaY = -0.05)
        if (calibrationRising(10, pov == 270)) calibration.adjustTruth(deltaX = -0.05)
        if (calibrationRising(11, pov == 90)) calibration.adjustTruth(deltaX = 0.05)

        calibration.periodic(timestampMs)
        robot.telemetry.putString("Calibration/Localization/TestType", calibration.testType.name)
        robot.telemetry.putNumber("Calibration/Localization/RunId", calibration.runId.toDouble())
        robot.telemetry.putBoolean("Calibration/Localization/Recording", calibration.continuousRecording)
        robot.telemetry.putNumber("Calibration/Localization/TruthX", calibration.truthX)
        robot.telemetry.putNumber("Calibration/Localization/TruthY", calibration.truthY)
        robot.telemetry.putNumber("Calibration/Localization/TruthHeadingRad", calibration.truthHeading)
        robot.telemetry.putNumber("Calibration/Localization/DroppedSamples", calibration.droppedSampleCount.toDouble())
    }

    override fun testExit() {
        localizationVisionTracker?.fusionEnabled = true
        localizationVisionTracker = null
        localizationCalibration?.close()
        localizationCalibration = null
    }

    private fun calibrationRising(index: Int, pressed: Boolean): Boolean {
        val rising = pressed && !calibrationButtonState[index]
        calibrationButtonState[index] = pressed
        return rising
    }

    private fun applyMechanismSafetyPolicy(source: String) {
        robot.store.dispatch(SetMechanismSafetyInhibit(!mechanismConfigurationValid))
        robot.telemetry.putBoolean("Safety/MechanismConfigurationValid", mechanismConfigurationValid)
        if (!mechanismConfigurationValid) {
            DriverStation.reportError(
                "ARES: mechanism configuration invalid during $source; outputs remain inhibited",
                false
            )
            robot.safeHardware()
        }
    }

    private fun latchMechanismAllStop() {
        runCatching { robot.store.dispatch(SetMechanismSafetyInhibit(true)) }
        robot.safeHardware()
    }

    override fun close() {
        var failure: Throwable? = null
        try {
            if (::robot.isInitialized) robot.close()
        } catch (error: Throwable) {
            failure = error
        }
        try {
            powerDistribution?.close()
        } catch (error: Throwable) {
            failure?.addSuppressed(error) ?: run { failure = error }
        } finally {
            powerDistribution = null
        }
        try {
            super.close()
        } catch (error: Throwable) {
            failure?.addSuppressed(error) ?: run { failure = error }
        }
        failure?.let { throw it }
    }

    // ── Simulation ──

    /** Advances dyn4j, dispatches simulated events/ground-truth pose, and publishes 3D state. */
    override fun simulationPeriodic() {
        if (!RobotBase.isSimulation()) return
        val simInstance = sim ?: return

        val now = com.areslib.util.RobotClock.currentTimeMillis() / 1000.0
        val dt = Math.min(now - lastSimTime, 0.05)
        lastSimTime = now

        // Step physics and dispatch any resulting actions (ball intake/shoot)
        val actions = simInstance.step(robot.store.state, dt)
        for (action in actions) {
            robot.store.dispatch(action)
        }

        // Dispatch pose update so the state has odometry
        val poseUpdate = simInstance.getPoseUpdate()
        robot.store.dispatch(poseUpdate)

        // Publish 3D visualization
        simInstance.publishVisualization(robot.store.state, robot.telemetry)
    }
}
