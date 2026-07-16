package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.state.RobotState
import com.areslib.action.RobotAction
import com.areslib.math.geometry.Pose2d
import com.areslib.math.geometry.Translation2d
import com.areslib.math.geometry.ChassisSpeeds
import com.areslib.control.assist.ShotResult
import com.areslib.control.assist.ShotSetup

class MarvinShooterSubsystem(private val store: Store) {
    private val shotSetup = ShotSetup(MarvinConfig.SHOT_CONFIG)
    
    private var lastFlywheelRpm = Double.NaN
    private var lastFlywheelActive: Boolean? = null
    private var lastCowlAngle = Double.NaN
    private var lastFeederSpeed = Double.NaN
    private var lastFloorSpeed = Double.NaN
    private var lastTransferActive: Boolean? = null


    val flywheelRPM: Double
        get() = store.state.superstructure.marvin.flywheel.velocityRpm

    val flywheelTargetRPM: Double
        get() = store.state.superstructure.marvin.flywheel.targetVelocityRpm

    val cowlAngleDegrees: Double
        get() = store.state.superstructure.marvin.cowl.angleDegrees

    val transferActive: Boolean
        get() = store.state.superstructure.marvin.transferActive

    fun spinUp(targetRpm: Double) {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        if (targetRpm != lastFlywheelRpm) {
            store.dispatch(SetFlywheelSpeed(targetRpm, timestamp))
            lastFlywheelRpm = targetRpm
        }
        if (lastFlywheelActive != true) {
            store.dispatch(SetFlywheelActive(true, timestamp))
            lastFlywheelActive = true
        }
    }

    fun shoot() {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        if (lastTransferActive != true) {
            store.dispatch(SetTransferActive(true, timestamp))
            lastTransferActive = true
        }
    }

    fun stop() {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        if (lastFlywheelActive != false) {
            store.dispatch(SetFlywheelActive(false, timestamp))
            lastFlywheelActive = false
        }
        if (lastTransferActive != false) {
            store.dispatch(SetTransferActive(false, timestamp))
            lastTransferActive = false
        }
    }

    fun setCowlAngle(degrees: Double) {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        if (degrees != lastCowlAngle) {
            store.dispatch(SetCowlAngle(degrees, timestamp))
            lastCowlAngle = degrees
        }
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
        
        shotSetup.calculate(currentPose, fieldSpeeds, targetTranslation, shotResult)
        
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        
        val targetRpm = shotResult.targetFlywheelRpm
        if (targetRpm != lastFlywheelRpm) {
            store.dispatch(SetFlywheelSpeed(targetRpm, timestamp))
            lastFlywheelRpm = targetRpm
        }
        if (lastFlywheelActive != true) {
            store.dispatch(SetFlywheelActive(true, timestamp))
            lastFlywheelActive = true
        }
        val targetCowl = shotResult.targetCowlAngleDegrees
        if (targetCowl != lastCowlAngle) {
            store.dispatch(SetCowlAngle(targetCowl, timestamp))
            lastCowlAngle = targetCowl
        }
        
        val headingError = shotResult.robotTargetHeadingRad - currentPose.heading.radians
        val wrappedError = com.areslib.math.wrapAngle(headingError)
        val kp = 4.0
        val rotation = wrappedError * kp + shotResult.angularVelocityFeedforwardRadPerSec
        
        val headingAligned = kotlin.math.abs(wrappedError) < 0.05
        val rpmAligned = kotlin.math.abs(store.state.superstructure.marvin.flywheel.velocityRpm - shotResult.targetFlywheelRpm) < 150.0
        
        val speed = if (headingAligned && rpmAligned) 10.0 else 0.0
        if (speed != lastFeederSpeed) {
            store.dispatch(SetFeederSpeed(speed, timestamp))
            lastFeederSpeed = speed
        }
        if (runFloorRollers) {
            if (speed != lastFloorSpeed) {
                store.dispatch(SetFloorSpeed(speed, timestamp))
                lastFloorSpeed = speed
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
        val targetRpm = shotSetup.interpolateRpm(dist)
        val targetCowl = shotSetup.interpolateCowl(dist)
        
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        if (targetRpm != lastFlywheelRpm) {
            store.dispatch(SetFlywheelSpeed(targetRpm, timestamp))
            lastFlywheelRpm = targetRpm
        }
        if (lastFlywheelActive != true) {
            store.dispatch(SetFlywheelActive(true, timestamp))
            lastFlywheelActive = true
        }
        if (targetCowl != lastCowlAngle) {
            store.dispatch(SetCowlAngle(targetCowl, timestamp))
            lastCowlAngle = targetCowl
        }
        
        val headingError = Math.atan2(targetTranslation.y - currentPose.y, targetTranslation.x - currentPose.x) - currentPose.heading.radians + Math.PI
        val wrappedError = com.areslib.math.wrapAngle(headingError)
        val kp = 4.0
        val rotation = wrappedError * kp
        
        val headingAligned = kotlin.math.abs(wrappedError) < 0.05
        val rpmAligned = kotlin.math.abs(store.state.superstructure.marvin.flywheel.velocityRpm - targetRpm) < 150.0
        val feederSpeed = if (headingAligned && rpmAligned) 10.0 else 0.0
        if (feederSpeed != lastFeederSpeed) {
            store.dispatch(SetFeederSpeed(feederSpeed, timestamp))
            lastFeederSpeed = feederSpeed
        }
        
        return rotation
    }
}

class MarvinIntakeSubsystem(private val store: Store) {
    val isDeployed: Boolean
        get() = store.state.superstructure.marvin.intake.isDeployed

    val pivotAngleDegrees: Double
        get() = store.state.superstructure.marvin.intake.pivotAngleDegrees

    val rollerSpeedRps: Double
        get() = store.state.superstructure.marvin.intake.rollerVelocityRps

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
        get() = store.state.superstructure.marvin.climber.extensionMeters

    val targetExtensionMeters: Double
        get() = store.state.superstructure.marvin.climber.targetExtensionMeters

    fun setTargetExtension(meters: Double) {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        store.dispatch(SetClimberExtension(meters, timestamp))
    }

    fun setVoltage(volts: Double) {
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()
        store.dispatch(SetClimberVoltage(volts, timestamp))
    }
}
