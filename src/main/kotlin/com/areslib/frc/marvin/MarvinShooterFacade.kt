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
    
    private val flywheelController = MarvinFlywheelController(store)
    private val cowlController = MarvinCowlController(store)
    private val feederController = MarvinFeederController(store)
    
    private val scratchSpeeds = ChassisSpeeds(0.0, 0.0, 0.0)

    val flywheelRPM: Double
        get() = flywheelController.flywheelRPM

    val flywheelTargetRPM: Double
        get() = flywheelController.flywheelTargetRPM

    val cowlAngleRotations: Double
        get() = cowlController.cowlAngleRotations

    val transferActive: Boolean
        get() = feederController.transferActive

    fun spinUp(targetRpm: Double) {
        flywheelController.spinUp(targetRpm)
    }

    fun shoot() {
        feederController.shoot()
    }

    fun stop() {
        flywheelController.stop()
        feederController.stop()
    }

    fun setCowlAngle(degrees: Double) {
        cowlController.setCowlAngle(degrees)
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
        flywheelController.spinUp(targetRpm)
        
        val targetCowl = shotResult.targetCowlAngleDegrees
        cowlController.setCowlAngle(targetCowl)
        
        val headingError = shotResult.robotTargetHeadingRad - currentPose.heading.radians
        val wrappedError = com.areslib.math.wrapAngle(headingError)
        val kp = 4.0
        val rotation = wrappedError * kp + shotResult.angularVelocityFeedforwardRadPerSec
        
        val headingAligned = kotlin.math.abs(wrappedError) < 0.05
        val rpmAligned = flywheelController.isRpmAligned(shotResult.targetFlywheelRpm)
        
        feederController.updateFeeders(rpmAligned, headingAligned, runFloorRollers)
        
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
        
        flywheelController.spinUp(targetRpm)
        cowlController.setCowlAngle(targetCowl)
        
        val headingError = Math.atan2(targetTranslation.y - currentPose.y, targetTranslation.x - currentPose.x) - currentPose.heading.radians + Math.PI
        val wrappedError = com.areslib.math.wrapAngle(headingError)
        val kp = 4.0
        val rotation = wrappedError * kp
        
        val headingAligned = kotlin.math.abs(wrappedError) < 0.05
        val rpmAligned = flywheelController.isRpmAligned(targetRpm)
        
        feederController.updateFeeders(rpmAligned, headingAligned, false)
        
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
