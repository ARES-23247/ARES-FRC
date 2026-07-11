package com.areslib.frc.marvin

import com.areslib.action.RobotAction

data class SetFlywheelSpeed @kotlin.jvm.JvmOverloads constructor(
    val rpm: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetCowlAngle @kotlin.jvm.JvmOverloads constructor(
    val degrees: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetIntakePivot @kotlin.jvm.JvmOverloads constructor(
    val deployed: Boolean,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetIntakeRollers @kotlin.jvm.JvmOverloads constructor(
    val speedRps: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetFeederSpeed @kotlin.jvm.JvmOverloads constructor(
    val speedRps: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetFloorSpeed @kotlin.jvm.JvmOverloads constructor(
    val speedRps: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetClimberVoltage @kotlin.jvm.JvmOverloads constructor(
    val volts: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SetClimberExtension @kotlin.jvm.JvmOverloads constructor(
    val meters: Double,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class SuperstructureSensorUpdate @kotlin.jvm.JvmOverloads constructor(
    val flywheelRpm: Double,
    val cowlAngle: Double,
    val intakeAngle: Double,
    val pieceDetected: Boolean,
    val floorVelocityRps: Double = 0.0,
    val climberExtensionMeters: Double = 0.0,
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class StartSlamtake(
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction

data class StopSlamtake(
    override val timestampMs: Long = com.areslib.util.RobotClock.currentTimeMillis()
) : RobotAction
