package com.areslib.frc.marvin

import com.areslib.Store

/**
 * Facade providing high-level operational commands for the Marvin intake superstructure.
 *
 * Dispatches deployment (pivot) and roller velocity setpoints to the Redux store
 * using change deduplication to minimize allocations on 50 Hz control loops.
 *
 * @param store The central Redux-style store containing global robot state.
 */
class MarvinIntakeSubsystem(store: Store) : MarvinControllerBase(store) {

    /** Current deployed state from the Redux store. */
    val isDeployed: Boolean
        get() = store.state.superstructure.marvin.intake.isDeployed

    /** Measured pivot angle in degrees from the latest sensor update. */
    val pivotAngleDegrees: Double
        get() = store.state.superstructure.marvin.intake.pivotAngleDegrees

    /** True when the latest pivot angle measurement is valid and fresh. */
    val pivotAngleValid: Boolean
        get() = store.state.superstructure.marvin.intake.pivotAngleValid

    /** Commanded target pivot angle in degrees. */
    val targetAngleDegrees: Double
        get() = store.state.superstructure.marvin.intake.targetAngleDegrees

    /** Measured roller velocity in RPS from the latest sensor update. */
    val rollerVelocityRps: Double
        get() = store.state.superstructure.marvin.intake.rollerVelocityRps

    /** Commanded target roller velocity in RPS. */
    val targetRollerVelocityRps: Double
        get() = store.state.superstructure.marvin.intake.targetRollerVelocityRps

    /** True when the intake is deployed and pivot angle is within alignment tolerance of 90 degrees. */
    val isDeployedAndAligned: Boolean
        get() = isDeployed && isPivotAligned(MarvinConfig.MechanismLimits.intakeDeployedDegrees)

    /**
     * Commands the intake pivot to the deployed position (90 degrees).
     */
    fun deploy() {
        setPivot(deployed = true)
    }

    /**
     * Commands the intake pivot to the stowed position (0 degrees).
     */
    fun stow() {
        setPivot(deployed = false)
    }

    /**
     * Commands the intake pivot deployed (`true`, 90 degrees) or stowed (`false`, 0 degrees).
     */
    fun setPivot(deployed: Boolean) {
        dispatchOnChange(
            store.state.superstructure.marvin.intake.isDeployed,
            deployed,
            ::SetIntakePivot
        ) {}
    }

    /**
     * Commands the intake roller speed in revolutions per second (RPS).
     */
    fun setRollers(speedRps: Double) {
        dispatchOnChange(
            store.state.superstructure.marvin.intake.targetRollerVelocityRps,
            speedRps,
            ::SetIntakeRollers
        ) {}
    }

    /**
     * Deploys the intake pivot and runs the rollers forward at the specified speed.
     *
     * @param speedRps Roller velocity in RPS, defaults to [DEFAULT_COLLECT_RPS].
     */
    fun collect(speedRps: Double = DEFAULT_COLLECT_RPS) {
        deploy()
        setRollers(speedRps)
    }

    /**
     * Stops the intake rollers without moving the pivot.
     */
    fun stopRollers() {
        setRollers(0.0)
    }

    /**
     * Stops the intake rollers and stows the intake pivot.
     */
    fun stopAndStow() {
        stopRollers()
        stow()
    }

    /**
     * Deploys the intake pivot and runs the rollers in reverse to clear jams.
     *
     * @param speedRps Roller reverse velocity in RPS, defaults to [DEFAULT_UNJAM_RPS].
     */
    fun unjam(speedRps: Double = DEFAULT_UNJAM_RPS) {
        deploy()
        val reverseSpeed = if (speedRps > 0.0) -speedRps else speedRps
        setRollers(reverseSpeed)
    }

    /**
     * Checks whether the intake pivot measurement is valid and within tolerance of the target angle.
     *
     * @param targetDegrees Target angle in degrees.
     * @param toleranceDegrees Allowable angular error in degrees.
     */
    fun isPivotAligned(
        targetDegrees: Double = store.state.superstructure.marvin.intake.targetAngleDegrees,
        toleranceDegrees: Double = PIVOT_ALIGNMENT_TOLERANCE_DEGREES
    ): Boolean {
        val intake = store.state.superstructure.marvin.intake
        return targetDegrees.isFinite() &&
            intake.pivotAngleValid &&
            intake.pivotAngleDegrees.isFinite() &&
            kotlin.math.abs(intake.pivotAngleDegrees - targetDegrees) <= toleranceDegrees
    }

    companion object {
        const val DEFAULT_COLLECT_RPS = 10.0
        const val DEFAULT_UNJAM_RPS = -5.0
        const val PIVOT_ALIGNMENT_TOLERANCE_DEGREES = 5.0
    }
}
