package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.state.RobotState
import com.areslib.action.RobotAction
import com.areslib.math.geometry.Pose2d
import com.areslib.math.geometry.Translation2d
import com.areslib.math.geometry.ChassisSpeeds
import com.areslib.control.assist.ShotResult
import com.areslib.control.assist.ShotSetup

/**
 * Facade providing high-level operational commands for the shooter superstructure.
 *
 * This subsystem controller orchestrates multiple underlying controllers (flywheel,
 * cowl, feeder) to deliver a single-responsibility interface for aiming and firing.
 * 
 * **Physical Units & Conventions:**
 * - Translational velocities: Meters per second ($m/s$).
 * - Angular velocities: Radians per second ($rad/s$).
 * - Heading: CCW-positive radians ($rad$).
 * - Angles/Rotations: Rotations or Degrees as specifically named.
 *
 * **Performance Guarantees:**
 * - Zero-GC Allocations in hot teleop and auto periodic update loops.
 *
 * @param store The central Redux-style store containing global robot state.
 */
class MarvinShooterSubsystem(private val store: Store) {
    private val shotSetup = ShotSetup(MarvinConfig.SHOT_CONFIG)
    
    private val flywheelController = MarvinFlywheelController(store)
    private val cowlController = MarvinCowlController(store)
    private val feederController = MarvinFeederController(store)
    
    private val scratchSpeeds = ChassisSpeeds(0.0, 0.0, 0.0)
    /**
     * Documentation for flywheelRPM
     */

    val flywheelRPM: Double
        get() = flywheelController.flywheelRPM
    /**
     * Documentation for flywheelTargetRPM
     */

    val flywheelTargetRPM: Double
        get() = flywheelController.flywheelTargetRPM
    /**
     * Documentation for cowlAngleRotations
     */

    val cowlAngleRotations: Double
        get() = cowlController.cowlAngleRotations
    /**
     * Documentation for transferActive
     */

    val transferActive: Boolean
        get() = feederController.transferActive
    /**
     * Documentation for spinUp
     */

    fun spinUp(targetRpm: Double) {
        flywheelController.spinUp(targetRpm)
    }
    /**
     * Documentation for shoot
     */

    fun shoot() {
        feederController.shoot()
    }
    /**
     * Documentation for stop
     */

    fun stop() {
        flywheelController.stop()
        feederController.stop()
    }
    /**
     * Documentation for setCowlAngle
     */

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
        /**
         * Documentation for driveState
         */
        val driveState = store.state.drive
        /**
         * Documentation for rx
         */
        val rx = driveState.xVelocityMetersPerSecond
        /**
         * Documentation for ry
         */
        val ry = driveState.yVelocityMetersPerSecond
        /**
         * Documentation for omega
         */
        val omega = driveState.angularVelocityRadiansPerSecond
        /**
         * Documentation for cos
         */
        
        val cos = currentPose.heading.cos
        /**
         * Documentation for sin
         */
        val sin = currentPose.heading.sin
        /**
         * Documentation for fieldVx
         */
        val fieldVx = rx * cos - ry * sin
        /**
         * Documentation for fieldVy
         */
        val fieldVy = rx * sin + ry * cos
        
        scratchSpeeds.vxMetersPerSecond = fieldVx
        scratchSpeeds.vyMetersPerSecond = fieldVy
        scratchSpeeds.omegaRadiansPerSecond = omega
        
        shotSetup.calculate(currentPose, scratchSpeeds, targetTranslation, shotResult)
        /**
         * Documentation for targetRpm
         */
        
        val targetRpm = shotResult.targetFlywheelRpm
        flywheelController.spinUp(targetRpm)
        /**
         * Documentation for targetCowl
         */
        
        val targetCowl = shotResult.targetCowlAngleDegrees
        cowlController.setCowlAngle(targetCowl)
        /**
         * Documentation for headingError
         */
        
        val headingError = shotResult.robotTargetHeadingRad - currentPose.heading.radians
        /**
         * Documentation for wrappedError
         */
        val wrappedError = com.areslib.math.wrapAngle(headingError)
        /**
         * Documentation for kp
         */
        val kp = 4.0
        /**
         * Documentation for rotation
         */
        val rotation = wrappedError * kp + shotResult.angularVelocityFeedforwardRadPerSec
        /**
         * Documentation for headingAligned
         */
        
        val headingAligned = kotlin.math.abs(wrappedError) < 0.05
        /**
         * Documentation for rpmAligned
         */
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
        /**
         * Documentation for dist
         */
        val dist = kotlin.math.hypot(currentPose.x - targetTranslation.x, currentPose.y - targetTranslation.y)
        /**
         * Documentation for targetRpm
         */
        val targetRpm = shotSetup.interpolateRpm(dist)
        /**
         * Documentation for targetCowl
         */
        val targetCowl = shotSetup.interpolateCowl(dist)
        
        flywheelController.spinUp(targetRpm)
        cowlController.setCowlAngle(targetCowl)
        /**
         * Documentation for headingError
         */
        
        val headingError = Math.atan2(targetTranslation.y - currentPose.y, targetTranslation.x - currentPose.x) - currentPose.heading.radians + Math.PI
        /**
         * Documentation for wrappedError
         */
        val wrappedError = com.areslib.math.wrapAngle(headingError)
        /**
         * Documentation for kp
         */
        val kp = 4.0
        /**
         * Documentation for rotation
         */
        val rotation = wrappedError * kp
        /**
         * Documentation for headingAligned
         */
        
        val headingAligned = kotlin.math.abs(wrappedError) < 0.05
        /**
         * Documentation for rpmAligned
         */
        val rpmAligned = flywheelController.isRpmAligned(targetRpm)
        
        feederController.updateFeeders(rpmAligned, headingAligned, false)
        
        return rotation
    }
}

