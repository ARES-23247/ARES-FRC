package com.areslib.frc

import com.areslib.action.RobotAction
import com.areslib.hardware.FlywheelIO
import com.areslib.hardware.CowlIO
import com.areslib.hardware.IntakeIO
import com.areslib.hardware.FeederIO
import com.areslib.state.DriveState
import com.areslib.subsystem.AresRobot
import com.areslib.subsystem.DriveSubsystem
import com.areslib.subsystem.SwerveDriveFacade
import com.areslib.telemetry.*
import com.areslib.frc.action.*
import com.areslib.frc.subsystem.*
import com.areslib.frc.state.marvinXIX
import com.areslib.frc.power.FrcPowerManager
import com.areslib.frc.telemetry.FrcTelemetryManager
import com.areslib.frc.vision.FrcVisionTracker

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.TalonFX
import com.areslib.hardware.vision.CompositeVisionIO
import frc.robot.generated.TunerConstants

/**
 * FRC Swerve Robot facade — the FRC mirror of FtcMecanumRobot.
 *
 * Extends AresRobot for the Store + superstructure subsystem facades, and delegates
 * peripheral tasks to dedicated managers (power, telemetry, vision).
 */
class FrcSwerveRobot(
    private val swerveIO: SwerveHardwareIO? = null,
    private val flywheelIO: FlywheelIO,
    private val cowlIO: CowlIO,
    private val intakeIO: IntakeIO,
    private val feederIO: FeederIO,
    private val floorIO: com.areslib.hardware.FloorIO = object : com.areslib.hardware.FloorIO {
        override fun setAppliedVoltage(volts: Double) {}
    },
    private val climberIO: com.areslib.hardware.ClimberIO = object : com.areslib.hardware.ClimberIO {
        override fun setTargetExtension(meters: Double) {}
        override fun setAppliedVoltage(volts: Double) {}
    },
    private val visionIO: com.areslib.hardware.vision.VisionIO? = null,
    private val isSimulation: Boolean = false,
    baseTelemetry: ITelemetry = FRCTelemetry(),
    private val isEnabledProvider: () -> Boolean = {
        try {
            edu.wpi.first.wpilibj.DriverStation.isEnabled()
        } catch (_: Throwable) {
            false
        }
    },
    private val robotModeProvider: () -> String = {
        try {
            when {
                edu.wpi.first.wpilibj.DriverStation.isAutonomous() -> "Auto"
                edu.wpi.first.wpilibj.DriverStation.isTeleop() -> "Teleop"
                edu.wpi.first.wpilibj.DriverStation.isTest() -> "Test"
                else -> "Disabled"
            }
        } catch (_: Throwable) {
            "Active"
        }
    }
) : AresRobot(
    initialState = com.areslib.state.RobotState(
        vision = com.areslib.state.VisionState(
            filterConfig = com.areslib.hardware.vision.VisionFilterConfig.frcDefaults()
        )
    ),
    reducer = com.areslib.frc.reducer.MarvinReducer::reduce
) {

    // Subsystem Facades
    val drive = DriveSubsystem(store)
    val swerveDrive = SwerveDriveFacade(store)

    val marvinShooter = MarvinShooterSubsystem(store)
    val marvinIntake = MarvinIntakeSubsystem(store)
    val marvinClimber = MarvinClimberSubsystem(store)

    // Modular managers
    val telemetryManager = FrcTelemetryManager(baseTelemetry, store)
    val powerManager = FrcPowerManager()
    val visionTracker = FrcVisionTracker(store, visionIO, swerveIO, isSimulation)

    // Alias properties for compatibility
    val telemetry: ITelemetry get() = telemetryManager.dataLoggingTelemetry
    val brownoutGuard get() = powerManager.brownoutGuard
    var batteryVoltageSupplier: () -> Double
        get() = powerManager.batteryVoltageSupplier
        set(value) { powerManager.batteryVoltageSupplier = value }

    private var wasBeached = false
    private val scratchSpeeds = DoubleArray(4)
    private val scratchCurrents = DoubleArray(4)

    init {
        RobotWebServer.start()
        RobotStatusTracker.isEnabled = false
        RobotStatusTracker.activeOpMode = "Init"

        // Register all devices with HardwareRegistry for automated logging, lifecycle refresh, and safety shutdown
        swerveIO?.let { com.areslib.hardware.HardwareRegistry.registerDevice("Swerve", it) }
        com.areslib.hardware.HardwareRegistry.registerDevice("Flywheel", flywheelIO)
        com.areslib.hardware.HardwareRegistry.registerDevice("Cowl", cowlIO)
        com.areslib.hardware.HardwareRegistry.registerDevice("Intake", intakeIO)
        com.areslib.hardware.HardwareRegistry.registerDevice("Feeder", feederIO)
        com.areslib.hardware.HardwareRegistry.registerDevice("Floor", floorIO)
        com.areslib.hardware.HardwareRegistry.registerDevice("Climber", climberIO)
        visionIO?.let { com.areslib.hardware.HardwareRegistry.registerDevice("Vision", it) }
    }

    /**
     * Coordinated frame update for FRC Swerve Drivetrain.
     *
     * 1. Reads hardware sensors → dispatches to Store
     * 2. Writes Store state → hardware outputs
     * 3. Publishes everything through unified pipeline (NT4 + CSV)
     *
     * @param gamepad1 Optional driver gamepad (use `controller.toState()`)
     * @param gamepad2 Optional operator gamepad
     */
    fun update(gamepad1: GamepadState? = null, gamepad2: GamepadState? = null) {
        try {
            // Refresh all hardware status signals in a batch
            com.areslib.hardware.HardwareRegistry.refreshAll()
            val isEnabled = isEnabledProvider()
            val mode = robotModeProvider()

            if (isEnabled) {
                RobotWebServer.stop()
            } else {
                RobotWebServer.start()
            }

            RobotStatusTracker.isEnabled = isEnabled
            RobotStatusTracker.activeOpMode = mode

            val timestamp = com.areslib.util.RobotClock.currentTimeMillis()

            // ── 1. READ: Hardware → Store ──
            if (!isSimulation && swerveIO != null) {
                val driveState = swerveIO.read()
                val currentlyBeached = isBeached
                val lastPose = store.state.drive.poseEstimator.estimatedPose
                val x = if (currentlyBeached) lastPose.x else driveState.odometryX
                val y = if (currentlyBeached) lastPose.y else driveState.odometryY

                if (wasBeached && !currentlyBeached) {
                    swerveIO.seedPose(lastPose)
                }
                wasBeached = currentlyBeached

                store.dispatch(RobotAction.PoseUpdate(
                    xMeters = x,
                    yMeters = y,
                    headingRadians = driveState.odometryHeading,
                    timestampMs = timestamp,
                    pitchDegrees = swerveIO.pitchDegrees,
                    rollDegrees = swerveIO.rollDegrees
                ))
            }

            // ── 1.5. READ: Vision → Store ──
            visionTracker.update(timestamp)

            // Read superstructure sensors
            store.dispatch(SuperstructureSensorUpdate(
                flywheelRpm = flywheelIO.velocityRpm,
                cowlAngle = cowlIO.angleDegrees,
                intakeAngle = intakeIO.pivotAngleDegrees,
                pieceDetected = feederIO.isBeamBroken,
                floorVelocityRps = floorIO.velocityRps,
                climberExtensionMeters = climberIO.extensionMeters,
                timestampMs = timestamp
            ))

            // ── 2. WRITE: Store → Hardware ──
            if (!isSimulation && swerveIO != null) {
                swerveIO.write(store.state.drive)
            }

            // ── 3. BROWNOUT PROTECTION ──
            val scale = powerManager.update()
            val batteryVoltage = powerManager.batteryVoltage

            // Apply power scaling to all superstructure outputs
            // (Drive swerve modules have their own voltage compensation via CTRE)
            val marvin = store.state.superstructure.marvinXIX
            flywheelIO.setVelocityRpm(marvin.flywheel.targetVelocityRpm * scale)
            cowlIO.setTargetAngle(marvin.cowl.targetAngleDegrees)

            val pivotAngle = marvin.intake.targetAngleDegrees
            intakeIO.setPivotAngle(pivotAngle)

            val targetRollerSpeed = marvin.intake.targetRollerVelocityRps
            intakeIO.setRollerVoltage((targetRollerSpeed / 10.0) * 12.0 * scale)

            val targetFeederSpeed = marvin.feeder.targetVelocityRps
            feederIO.setAppliedVoltage((targetFeederSpeed / 12.0) * 12.0 * scale)

            val targetFloorSpeed = marvin.floor.targetVelocityRps
            floorIO.setAppliedVoltage((targetFloorSpeed / 12.0) * 12.0 * scale)

            val targetClimberVoltage = marvin.climber.targetVoltage
            climberIO.setAppliedVoltage(targetClimberVoltage * scale)

            // ── 4. PUBLISH: Everything → NT4 + CSV ──
            telemetryManager.publish(gamepad1, gamepad2)
            telemetryManager.logBrownout(powerManager.brownoutGuard, batteryVoltage)
            com.areslib.hardware.HardwareRegistry.publishAll(telemetry)

        } catch (e: Throwable) {
            System.err.println("FrcSwerveRobot: Exception in update loop: ${e.message}")
            e.printStackTrace()
            safeHardware()
        }
    }

    fun safeHardware() {
        try {
            com.areslib.hardware.HardwareRegistry.safeAll()
        } catch (ex: Throwable) {
            System.err.println("FrcSwerveRobot: Failed to apply safety stop: ${ex.message}")
        }
    }

    val isBeached: Boolean
        get() {
            if (isSimulation || swerveIO == null) return false
            val pitch = swerveIO.pitchDegrees
            val roll = swerveIO.rollDegrees
            
            // Tilted chassis represents climbing/riding up on a note/ball.
            // 8.0 degrees prevents false positives from normal suspension travel (braking/acceleration).
            val isTilted = Math.abs(pitch) > 8.0 || Math.abs(roll) > 8.0
            
            // Loss of traction: high speed but very low current draw
            swerveIO.getModuleSpeeds(scratchSpeeds)
            swerveIO.getCurrents(scratchCurrents)
            var slipCount = 0
            for (i in 0..3) {
                if (Math.abs(scratchSpeeds[i]) > 1.5 && Math.abs(scratchCurrents[i]) < 8.0) {
                    slipCount++
                }
            }
            return isTilted && slipCount >= 2
        }

    companion object {
        /**
         * Factory function to initialize the Marvin XIX physical hardware.
         */
        fun createPhysicalMarvinXIX(): FrcSwerveRobot {
            val can2Bus = CANBus("CAN2")

            // Marvin 19 Physical Hardware on "CAN2" high-speed bus
            val leftMasterFX = TalonFX(9, can2Bus)
            val leftFollowerFX = TalonFX(10, can2Bus)
            val rightMasterFX = TalonFX(11, can2Bus)
            val rightFollowerFX = TalonFX(12, can2Bus)
            val cowlFX = TalonFX(13, can2Bus)
            val pivotFX = TalonFX(14, can2Bus)
            val rollerFX = TalonFX(15, can2Bus)
            val floorFX = TalonFX(16, can2Bus)
            val climberFX = TalonFX(19, can2Bus)
            val feederFX = TalonFX(20, can2Bus)

            // Initialize CTRE SwerveDrivetrain using Tuner X constants
            val ctreDrivetrain = TunerConstants.TunerSwerveDrivetrain(
                TunerConstants.DrivetrainConstants,
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight
            )
            val swerveIO = FRCSwerveHardwareIO(ctreDrivetrain)

            // Initialize Limelight cameras
            val limelightShooter = FrcLimelightIO("limelight-shooter")
            val limelightBack = FrcLimelightIO("limelight-back")
            val compositeVision = CompositeVisionIO(listOf(limelightShooter, limelightBack))

            return FrcSwerveRobot(
                swerveIO = swerveIO,
                flywheelIO = FRCFlywheelHardwareIO(leftMasterFX, leftFollowerFX, rightMasterFX, rightFollowerFX),
                cowlIO = FRCCowlHardwareIO(cowlFX),
                intakeIO = FRCIntakeHardwareIO(pivotFX, rollerFX),
                feederIO = FRCFeederHardwareIO(feederFX),
                floorIO = FRCFloorHardwareIO(floorFX),
                climberIO = FRCClimberHardwareIO(climberFX),
                visionIO = compositeVision,
                isSimulation = false
            )
        }
    }

    /**
     * Gracefully shuts down background logging threads and telemetry.
     */
    fun close() {
        RobotStatusTracker.isEnabled = false
        RobotWebServer.stop()
        telemetryManager.close()
        com.areslib.hardware.HardwareRegistry.closeAll()
        try {
            val ftcMotorClass = Class.forName("com.areslib.ftc.hardware.FtcMotor")
            val unregisterAllMethod = ftcMotorClass.getMethod("unregisterAll")
            unregisterAllMethod.invoke(null)
        } catch (_: Exception) {}
    }
}
