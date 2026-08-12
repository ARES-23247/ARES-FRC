package com.areslib.frc.marvin

import com.areslib.Store

/** Coordinates the feeder transfer latch and optional floor-roller assist. */
class MarvinFeederController(store: Store) : MarvinControllerBase(store) {
    private var transferStartTimeMs = NOT_STARTED
    private var transferConsumed = false

    /** True after a shot transfer has been explicitly started and before cancellation/timeout. */
    val transferActive: Boolean
        get() = store.state.superstructure.marvin.transferActive

    /** Ends the current trigger cycle so a later press may authorize one new transfer. */
    fun cancelTransfer() {
        transferStartTimeMs = NOT_STARTED
        transferConsumed = false
        stopOutputs()
    }

    private fun stopOutputs() {
        dispatchOnChange(store.state.superstructure.marvin.transferActive, false, ::SetTransferActive) {}
        dispatchOnChange(store.state.superstructure.marvin.feeder.targetVelocityRps, 0.0, ::SetFeederSpeed) {}
        dispatchOnChange(store.state.superstructure.marvin.floor.targetVelocityRps, 0.0, ::SetFloorSpeed) {}
    }

    /**
     * Applies the heading/RPM firing interlock.
     *
     * A transfer already in progress is allowed to finish even if alignment moves out
     * of tolerance. [runFloorRollers] controls whether the floor mirrors feeder speed.
     */
    fun updateFeeders(
        rpmAligned: Boolean,
        headingAligned: Boolean,
        cowlReady: Boolean,
        runFloorRollers: Boolean = false
    ) {
        val nowMs = com.areslib.util.RobotClock.currentTimeMillis()
        val canStartTransfer = headingAligned && rpmAligned && cowlReady
        if (transferActive && transferStartTimeMs == NOT_STARTED) transferStartTimeMs = nowMs
        if (transferStartTimeMs != NOT_STARTED) {
            val elapsedMs = nowMs - transferStartTimeMs
            if (elapsedMs < 0L || elapsedMs >= TRANSFER_DURATION_MS) {
                transferStartTimeMs = NOT_STARTED
                transferConsumed = true
                stopOutputs()
                return
            }
        }
        if (canStartTransfer && !transferActive && !transferConsumed) {
            store.dispatch(SetTransferActive(true))
            transferStartTimeMs = nowMs
        }
        val speed = if (transferActive) {
            MarvinConfig.FEEDER_SHOOT_SPEED_RPS
        } else {
            0.0
        }
        dispatchOnChange(store.state.superstructure.marvin.feeder.targetVelocityRps, speed, ::SetFeederSpeed) {}

        val floorSpeed = if (runFloorRollers) speed else 0.0
        dispatchOnChange(store.state.superstructure.marvin.floor.targetVelocityRps, floorSpeed, ::SetFloorSpeed) {}
    }

    private companion object {
        const val NOT_STARTED = -1L
        const val TRANSFER_DURATION_MS = 450L
    }
}
