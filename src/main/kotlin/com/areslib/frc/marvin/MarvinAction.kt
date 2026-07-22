package com.areslib.frc.marvin

import com.areslib.action.RobotAction

data class SetFlywheelSpeed @kotlin.jvm.JvmOverloads constructor(
    /**
     * Documentation for rpm
     */
    val rpm: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetCowlAngle @kotlin.jvm.JvmOverloads constructor(
    /**
     * Documentation for rotations
     */
    val rotations: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetFlywheelActive @kotlin.jvm.JvmOverloads constructor(
    /**
     * Documentation for active
     */
    val active: Boolean,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetTransferActive @kotlin.jvm.JvmOverloads constructor(
    /**
     * Documentation for active
     */
    val active: Boolean,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetInventoryCount @kotlin.jvm.JvmOverloads constructor(
    /**
     * Documentation for count
     */
    val count: Int,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetIntakePivot @kotlin.jvm.JvmOverloads constructor(
    /**
     * Documentation for deployed
     */
    val deployed: Boolean,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetIntakeRollers @kotlin.jvm.JvmOverloads constructor(
    /**
     * Documentation for speedRps
     */
    val speedRps: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetFeederSpeed @kotlin.jvm.JvmOverloads constructor(
    /**
     * Documentation for speedRps
     */
    val speedRps: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetFloorSpeed @kotlin.jvm.JvmOverloads constructor(
    /**
     * Documentation for speedRps
     */
    val speedRps: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetClimberVoltage @kotlin.jvm.JvmOverloads constructor(
    /**
     * Documentation for volts
     */
    val volts: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetClimberExtension @kotlin.jvm.JvmOverloads constructor(
    /**
     * Documentation for meters
     */
    val meters: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SuperstructureSensorUpdate @kotlin.jvm.JvmOverloads constructor(
    /**
     * Documentation for flywheelRpm
     */
    val flywheelRpm: Double,
    /**
     * Documentation for cowlAngleRotations
     */
    val cowlAngleRotations: Double,
    /**
     * Documentation for intakeAngle
     */
    val intakeAngle: Double,
    /**
     * Documentation for pieceDetected
     */
    val pieceDetected: Boolean,
    /**
     * Documentation for floorVelocityRps
     */
    val floorVelocityRps: Double = 0.0,
    /**
     * Documentation for climberExtensionMeters
     */
    val climberExtensionMeters: Double = 0.0,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class StartSlamtake(
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class StopSlamtake(
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction
