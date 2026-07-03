package com.areslib.frc.subsystem

import com.areslib.subsystem.Store
import com.areslib.state.SuperstructureMode
import com.areslib.frc.action.*
import com.areslib.action.RobotAction
import com.areslib.frc.state.marvinXIX
import com.areslib.math.Pose2d
import com.areslib.math.Translation2d
import com.areslib.math.ChassisSpeeds
import com.areslib.control.ShotResult
import com.areslib.control.ShotSetup

class MarvinShooterSubsystem(private val store: Store) {
    val mode: SuperstructureMode
        get() = store.state.superstructure.mode

    val flywheelRPM: Double
        get() = store.state.superstructure.marvinXIX.flywheel.velocityRpm

    val flywheelTargetRPM: Double
        get() = store.state.superstructure.marvinXIX.flywheel.targetVelocityRpm

    val cowlAngleDegrees: Double
        get() = store.state.superstructure.marvinXIX.cowl.angleDegrees

    val transferActive: Boolean
        get() = store.state.superstructure.transferActive

    fun spinUp(targetRpm: Double) {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        store.dispatch(SetFlywheelSpeed(targetRpm, timestamp))
        store.dispatch(RobotAction.SetFlywheelActive(true, timestamp))
    }

    fun shoot() {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        store.dispatch(RobotAction.SetTransferActive(true, timestamp))
    }

    fun stop() {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        store.dispatch(RobotAction.SetFlywheelActive(false, timestamp))
        store.dispatch(RobotAction.SetTransferActive(false, timestamp))
    }

    fun setCowlAngle(degrees: Double) {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        store.dispatch(SetCowlAngle(degrees, timestamp))
    }

    /**
     * Calculates SOTM parameters, dispatches target speeds/angles, and returns target rotation command.
     */
    fun updateShootOnTheMove(
        currentPose: Pose2d,
        targetTranslation: Translation2d,
        shotResult: ShotResult,
        runFloorRollers: Boolean = false
    ): Double {
        val driveState = store.state.drive
        val rx = driveState.xVelocityMetersPerSecond
        val ry = driveState.yVelocityMetersPerSecond
        val omega = driveState.angularVelocityRadiansPerSecond
        
        val cos = currentPose.heading.cos
        val sin = currentPose.heading.sin
        val fieldVx = rx * cos - ry * sin
        val fieldVy = rx * sin + ry * cos
        
        val fieldSpeeds = ChassisSpeeds(fieldVx, fieldVy, omega)
        
        ShotSetup.calculate(currentPose, fieldSpeeds, targetTranslation, shotResult)
        
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        store.dispatch(SetFlywheelSpeed(shotResult.targetFlywheelRpm, timestamp))
        store.dispatch(RobotAction.SetFlywheelActive(true, timestamp))
        store.dispatch(SetCowlAngle(shotResult.targetCowlAngleDegrees, timestamp))
        
        val headingError = shotResult.robotTargetHeadingRad - currentPose.heading.radians
        val wrappedError = com.areslib.math.InputMath.wrapAngle(headingError)
        val kp = 4.0
        val rotation = wrappedError * kp + shotResult.angularVelocityFeedforwardRadPerSec
        
        val headingAligned = Math.abs(wrappedError) < 0.05
        val rpmAligned = Math.abs(store.state.superstructure.marvinXIX.flywheel.velocityRpm - shotResult.targetFlywheelRpm) < 150.0
        
        if (headingAligned && rpmAligned) {
            store.dispatch(SetFeederSpeed(10.0, timestamp))
            if (runFloorRollers) {
                store.dispatch(SetFloorSpeed(10.0, timestamp))
            }
        } else {
            store.dispatch(SetFeederSpeed(0.0, timestamp))
            if (runFloorRollers) {
                store.dispatch(SetFloorSpeed(0.0, timestamp))
            }
        }
        
        return rotation
    }

    /**
     * Calculates static shooting parameters and dispatches targets.
     */
    fun updateStaticShoot(
        currentPose: Pose2d,
        targetTranslation: Translation2d
    ): Double {
        val dist = kotlin.math.hypot(currentPose.x - targetTranslation.x, currentPose.y - targetTranslation.y)
        val targetRpm = ShotSetup.interpolateRpm(dist)
        val targetCowl = ShotSetup.interpolateCowl(dist)
        
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        store.dispatch(SetFlywheelSpeed(targetRpm, timestamp))
        store.dispatch(RobotAction.SetFlywheelActive(true, timestamp))
        store.dispatch(SetCowlAngle(targetCowl, timestamp))
        
        val headingError = Math.atan2(targetTranslation.y - currentPose.y, targetTranslation.x - currentPose.x) - currentPose.heading.radians + Math.PI
        val wrappedError = com.areslib.math.InputMath.wrapAngle(headingError)
        val kp = 4.0
        val rotation = wrappedError * kp
        
        val headingAligned = Math.abs(wrappedError) < 0.05
        val rpmAligned = Math.abs(store.state.superstructure.marvinXIX.flywheel.velocityRpm - targetRpm) < 150.0
        if (headingAligned && rpmAligned) {
            store.dispatch(SetFeederSpeed(10.0, timestamp))
        } else {
            store.dispatch(SetFeederSpeed(0.0, timestamp))
        }
        
        return rotation
    }
}

class MarvinIntakeSubsystem(private val store: Store) {
    val isDeployed: Boolean
        get() = store.state.superstructure.marvinXIX.intake.isDeployed

    val pivotAngleDegrees: Double
        get() = store.state.superstructure.marvinXIX.intake.pivotAngleDegrees

    val rollerSpeedRps: Double
        get() = store.state.superstructure.marvinXIX.intake.rollerVelocityRps

    fun deploy() {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        store.dispatch(SetIntakePivot(deployed = true, timestampMs = timestamp))
    }

    fun retract() {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        store.dispatch(SetIntakePivot(deployed = false, timestampMs = timestamp))
    }

    fun setRollerSpeed(rps: Double) {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        store.dispatch(SetIntakeRollers(speedRps = rps, timestampMs = timestamp))
    }
}

class MarvinClimberSubsystem(private val store: Store) {
    val extensionMeters: Double
        get() = store.state.superstructure.marvinXIX.climber.extensionMeters

    val targetExtensionMeters: Double
        get() = store.state.superstructure.marvinXIX.climber.targetExtensionMeters

    fun setTargetExtension(meters: Double) {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        store.dispatch(SetClimberExtension(meters, timestamp))
    }

    fun setVoltage(volts: Double) {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        store.dispatch(SetClimberVoltage(volts, timestamp))
    }
}
