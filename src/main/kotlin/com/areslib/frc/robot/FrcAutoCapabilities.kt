package com.areslib.frc.robot

import com.areslib.action.RobotAction
import com.areslib.frc.marvin.MarvinConfig
import com.areslib.frc.marvin.marvin
import com.areslib.frc.marvin.SetFeederSpeed
import com.areslib.frc.marvin.SetFloorSpeed
import com.areslib.frc.marvin.SetFlywheelActive
import com.areslib.frc.marvin.SetFlywheelSpeed
import com.areslib.frc.marvin.SetIntakePivot
import com.areslib.frc.marvin.SetIntakeRollers
import com.areslib.frc.marvin.SetTransferActive
import com.areslib.frc.generated.GeneratedAresProjectCapabilities
import com.areslib.pathing.CommandKey
import com.areslib.pathing.NamedCommandDescriptor
import com.areslib.pathing.NamedCommands
import com.areslib.sequencer.Task
import com.areslib.state.RobotState

/**
 * Marvin actions available to generated ARES routines and legacy import markers.
 *
 * The checked-in `.ares/action-catalog.json` drives generated type safety. These descriptors keep
 * the existing named-marker adapter discoverable during migration. Every factory creates a fresh
 * task because task lifecycle state may never be shared between marker invocations or runs.
 */
object FrcAutoCapabilities : GeneratedAresProjectCapabilities {
    val INTAKE_COLLECT = NamedCommandDescriptor(
        key = CommandKey("intake.collect"),
        displayName = "Collect note",
        description = "Deploys the intake and runs the intake and floor rollers.",
        category = "Intake"
    )
    val INTAKE_STOP = NamedCommandDescriptor(
        key = CommandKey("intake.stop"),
        displayName = "Stop intake",
        description = "Stops the intake and floor rollers without moving the pivot.",
        category = "Intake"
    )
    val INTAKE_STOW = NamedCommandDescriptor(
        key = CommandKey("intake.stow"),
        displayName = "Stow intake",
        description = "Stops the rollers and retracts the intake pivot.",
        category = "Intake"
    )
    val SHOOTER_PREPARE = NamedCommandDescriptor(
        key = CommandKey("shooter.prepare"),
        displayName = "Prepare shooter",
        description = "Spins the flywheel to the autonomous shooting preset.",
        category = "Shooter"
    )
    val SHOOTER_FEED_WHEN_READY = NamedCommandDescriptor(
        key = CommandKey("shooter.feedWhenReady"),
        displayName = "Shoot when ready",
        description = "Waits up to two seconds for fresh aligned flywheel RPM and cowl position, then feeds the note.",
        category = "Shooter"
    )
    val SHOOTER_STOP = NamedCommandDescriptor(
        key = CommandKey("shooter.stop"),
        displayName = "Stop shooter",
        description = "Stops the flywheel, feeder, and floor roller and clears the transfer latch.",
        category = "Shooter"
    )

    val descriptors: List<NamedCommandDescriptor> = listOf(
        INTAKE_COLLECT,
        INTAKE_STOP,
        INTAKE_STOW,
        SHOOTER_PREPARE,
        SHOOTER_FEED_WHEN_READY,
        SHOOTER_STOP
    )

    /** Registers or replaces all FRC autonomous task factories. */
    fun register() {
        NamedCommands.register(INTAKE_COLLECT) { actionIntakeCollect() }
        NamedCommands.register(INTAKE_STOP) { actionIntakeStop() }
        NamedCommands.register(INTAKE_STOW) { actionIntakeStow() }
        NamedCommands.register(SHOOTER_PREPARE) { actionShooterPrepare() }
        NamedCommands.register(SHOOTER_FEED_WHEN_READY) { actionShooterFeedWhenReady() }
        NamedCommands.register(SHOOTER_STOP) { actionShooterStop() }
    }

    override fun actionIntakeCollect(): Task = InstantAutoActionsTask(INTAKE_COLLECT.displayName) {
        listOf(
            SetIntakePivot(deployed = true),
            SetIntakeRollers(INTAKE_ROLLER_RPS),
            SetFloorSpeed(FLOOR_ROLLER_RPS)
        )
    }

    override fun actionIntakeStop(): Task = InstantAutoActionsTask(INTAKE_STOP.displayName) {
        listOf(SetIntakeRollers(0.0), SetFloorSpeed(0.0))
    }

    override fun actionIntakeStow(): Task = InstantAutoActionsTask(INTAKE_STOW.displayName) {
        listOf(SetIntakeRollers(0.0), SetFloorSpeed(0.0), SetIntakePivot(deployed = false))
    }

    override fun actionShooterPrepare(): Task = InstantAutoActionsTask(SHOOTER_PREPARE.displayName) {
        listOf(SetFlywheelSpeed(AUTO_SHOT_RPM), SetFlywheelActive(active = true))
    }

    override fun actionShooterFeedWhenReady(): Task = FeedWhenReadyTask()

    override fun actionShooterStop(): Task =
        InstantAutoActionsTask(SHOOTER_STOP.displayName, ::shooterStopActions)

    override fun conditionShooterReady(): (RobotState) -> Boolean = ::flywheelIsReady

    internal fun allStopActions(): List<RobotAction> = buildList {
        addAll(shooterStopActions())
        add(SetIntakeRollers(0.0))
        add(SetIntakePivot(deployed = false))
    }

    private fun shooterStopActions(): List<RobotAction> = listOf(
        SetFlywheelSpeed(0.0),
        SetFlywheelActive(active = false),
        SetFeederSpeed(0.0),
        SetFloorSpeed(0.0),
        SetTransferActive(active = false)
    )

    private class InstantAutoActionsTask(
        override val name: String,
        private val actions: () -> List<RobotAction>
    ) : Task {
        private var dispatched = false

        override fun initialize(state: RobotState): List<RobotAction> {
            super.initialize(state)
            dispatched = true
            return actions()
        }

        override fun isCompleted(state: RobotState, elapsedMs: Long): Boolean = dispatched

        override fun releaseRuntimeState() {
            dispatched = false
            super.releaseRuntimeState()
        }
    }

    /** Bounded, fail-closed readiness gate for autonomous note transfer. */
    private class FeedWhenReadyTask : Task {
        override val name: String = SHOOTER_FEED_WHEN_READY.displayName
        private var feedIssued = false

        override fun initialize(state: RobotState): List<RobotAction> {
            super.initialize(state)
            feedIssued = false
            return emptyList()
        }

        override fun isCompleted(state: RobotState, elapsedMs: Long): Boolean =
            feedIssued || elapsedMs >= FEED_READY_TIMEOUT_MS

        override fun execute(state: RobotState, elapsedMs: Long): List<RobotAction> {
            super.execute(state, elapsedMs)
            if (feedIssued || !flywheelIsReady(state)) return emptyList()
            feedIssued = true
            return listOf(
                SetTransferActive(active = true),
                SetFeederSpeed(MarvinConfig.FEEDER_SHOOT_SPEED_RPS),
                SetFloorSpeed(MarvinConfig.FEEDER_SHOOT_SPEED_RPS)
            )
        }

        override fun end(state: RobotState, interrupted: Boolean): List<RobotAction> {
            val actions = if (interrupted || !feedIssued) {
                listOf(SetTransferActive(active = false), SetFeederSpeed(0.0), SetFloorSpeed(0.0))
            } else {
                emptyList()
            }
            super.end(state, interrupted)
            return actions
        }

        override fun releaseRuntimeState() {
            feedIssued = false
            super.releaseRuntimeState()
        }

    }

    internal fun flywheelIsReady(state: RobotState): Boolean {
        val flywheel = state.superstructure.marvin.flywheel
        val cowl = state.superstructure.marvin.cowl
        return flywheel.velocityValid &&
            flywheel.targetVelocityRpm > MINIMUM_READY_RPM &&
            kotlin.math.abs(flywheel.velocityRpm - flywheel.targetVelocityRpm) < RPM_TOLERANCE &&
            cowl.angleValid &&
            cowl.angleRotations.isFinite() &&
            cowl.targetAngleRotations.isFinite() &&
            kotlin.math.abs(cowl.angleRotations - cowl.targetAngleRotations) <= COWL_TOLERANCE_ROTATIONS
    }

    private const val AUTO_SHOT_RPM = 4_000.0
    private const val INTAKE_ROLLER_RPS = 15.0
    private const val FLOOR_ROLLER_RPS = 10.0
    private const val MINIMUM_READY_RPM = 100.0
    private const val RPM_TOLERANCE = 150.0
    private const val COWL_TOLERANCE_ROTATIONS = 0.05
    private const val FEED_READY_TIMEOUT_MS = 2_000L
}
