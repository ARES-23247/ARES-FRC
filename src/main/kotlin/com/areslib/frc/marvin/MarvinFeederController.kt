package com.areslib.frc.marvin

import com.areslib.Store

/** Coordinates the feeder transfer latch and optional floor-roller assist. */
class MarvinFeederController(store: Store) : MarvinControllerBase(store) {

    /** True after a shot transfer has been explicitly started and before [stop]. */
    val transferActive: Boolean
        get() = store.state.superstructure.marvin.transferActive

    /** Latches transfer active and commands the calibrated feeder shooting speed. */
    fun shoot() {
        dispatchOnChange(store.state.superstructure.marvin.transferActive, true, ::SetTransferActive) {}
        dispatchOnChange(store.state.superstructure.marvin.feeder.targetVelocityRps, MarvinConfig.FEEDER_SHOOT_SPEED_RPS, ::SetFeederSpeed) {}
    }

    /** Clears the transfer latch and stops both feeder and floor targets. */
    fun stop() {
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
        val canStartTransfer = headingAligned && rpmAligned && cowlReady
        if (canStartTransfer && !transferActive) {
            store.dispatch(SetTransferActive(true))
        }
        val speed = if (canStartTransfer || transferActive) {
            MarvinConfig.FEEDER_SHOOT_SPEED_RPS
        } else {
            0.0
        }
        dispatchOnChange(store.state.superstructure.marvin.feeder.targetVelocityRps, speed, ::SetFeederSpeed) {}

        val floorSpeed = if (runFloorRollers) speed else 0.0
        dispatchOnChange(store.state.superstructure.marvin.floor.targetVelocityRps, floorSpeed, ::SetFloorSpeed) {}
    }
}
