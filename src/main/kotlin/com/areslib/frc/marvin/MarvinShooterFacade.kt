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
    private val scratchSpeeds = ChassisSpeeds(0.0, 0.0, 0.0)

    private inline fun <T> dispatchOnChange(
        current: T?,
        target: T,
        actionFactory: (T, Long) -> RobotAction,
        updateCurrent: (T) -> Unit
    ) {
        if (current != target) {
            store.dispatch(actionFactory(target, com.areslib.util.RobotClock.currentTimeMillis()))
            updateCurrent(target)
        }
    }


    val flywheelRPM: Double
        get() = store.state.superstructure.marvin.flywheel.velocityRpm

    val flywheelTargetRPM: Double
        get() = store.state.superstructure.marvin.flywheel.targetVelocityRpm

    val cowlAngleRotations: Double
        get() = store.state.superstructure.marvin.cowl.angleRotations

    val transferActive: Boolean
        get() = store.state.superstructure.marvin.transferActive

    fun spinUp(targetRpm: Double) {
        dispatchOnChange(lastFlywheelRpm, targetRpm, ::SetFlywheelSpeed) { lastFlywheelRpm = it }
        dispatchOnChange(lastFlywheelActive, true, ::SetFlywheelActive) { lastFlywheelActive = it }
    }

    fun shoot() {
        dispatchOnChange(lastTransferActive, true, ::SetTransferActive) { lastTransferActive = it }
    }

    fun stop() {
        dispatchOnChange(lastFlywheelActive, false, ::SetFlywheelActive) { lastFlywheelActive = it }
        dispatchOnChange(lastTransferActive, false, ::SetTransferActive) { lastTransferActive = it }
    }

    fun setCowlAngle(degrees: Double) {
        dispatchOnChange(lastCowlAngle, degrees, ::SetCowlAngle) { lastCowlAngle = it }
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
        
        scratchSpeeds.vxMetersPerSecond = fieldVx
        scratchSpeeds.vyMetersPerSecond = fieldVy
        scratchSpeeds.omegaRadiansPerSecond = omega
        
        shotSetup.calculate(currentPose, scratchSpeeds, targetTranslation, shotResult)
        
        val targetRpm = shotResult.targetFlywheelRpm
        dispatchOnChange(lastFlywheelRpm, targetRpm, ::SetFlywheelSpeed) { lastFlywheelRpm = it }
        dispatchOnChange(lastFlywheelActive, true, ::SetFlywheelActive) { lastFlywheelActive = it }
        
        val targetCowl = shotResult.targetCowlAngleDegrees
        dispatchOnChange(lastCowlAngle, targetCowl, ::SetCowlAngle) { lastCowlAngle = it }
        
        val headingError = shotResult.robotTargetHeadingRad - currentPose.heading.radians
        val wrappedError = com.areslib.math.wrapAngle(headingError)
        val kp = 4.0
        val rotation = wrappedError * kp + shotResult.angularVelocityFeedforwardRadPerSec
        
        val headingAligned = kotlin.math.abs(wrappedError) < 0.05
        val rpmAligned = kotlin.math.abs(store.state.superstructure.marvin.flywheel.velocityRpm - shotResult.targetFlywheelRpm) < 150.0
        
        val speed = if (headingAligned && rpmAligned) 10.0 else 0.0
        dispatchOnChange(lastFeederSpeed, speed, ::SetFeederSpeed) { lastFeederSpeed = it }
        
        if (runFloorRollers) {
            dispatchOnChange(lastFloorSpeed, speed, ::SetFloorSpeed) { lastFloorSpeed = it }
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
        
        dispatchOnChange(lastFlywheelRpm, targetRpm, ::SetFlywheelSpeed) { lastFlywheelRpm = it }
        dispatchOnChange(lastFlywheelActive, true, ::SetFlywheelActive) { lastFlywheelActive = it }
        dispatchOnChange(lastCowlAngle, targetCowl, ::SetCowlAngle) { lastCowlAngle = it }
        
        val headingError = Math.atan2(targetTranslation.y - currentPose.y, targetTranslation.x - currentPose.x) - currentPose.heading.radians + Math.PI
        val wrappedError = com.areslib.math.wrapAngle(headingError)
        val kp = 4.0
        val rotation = wrappedError * kp
        
        val headingAligned = kotlin.math.abs(wrappedError) < 0.05
        val rpmAligned = kotlin.math.abs(store.state.superstructure.marvin.flywheel.velocityRpm - targetRpm) < 150.0
        val feederSpeed = if (headingAligned && rpmAligned) 10.0 else 0.0
        dispatchOnChange(lastFeederSpeed, feederSpeed, ::SetFeederSpeed) { lastFeederSpeed = it }
        
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
