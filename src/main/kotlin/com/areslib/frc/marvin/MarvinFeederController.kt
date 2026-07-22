package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.action.RobotAction

class MarvinFeederController(private val store: Store) {
    private var lastFeederSpeed = Double.NaN
    private var lastFloorSpeed = Double.NaN
    private var lastTransferActive: Boolean? = null

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

    val transferActive: Boolean
        get() = store.state.superstructure.marvin.transferActive

    fun shoot() {
        dispatchOnChange(lastTransferActive, true, ::SetTransferActive) { lastTransferActive = it }
    }

    fun stop() {
        dispatchOnChange(lastTransferActive, false, ::SetTransferActive) { lastTransferActive = it }
    }

    fun updateFeeders(rpmAligned: Boolean, headingAligned: Boolean, runFloorRollers: Boolean = false) {
        val speed = if (headingAligned && rpmAligned) 10.0 else 0.0
        dispatchOnChange(lastFeederSpeed, speed, ::SetFeederSpeed) { lastFeederSpeed = it }
        
        if (runFloorRollers) {
            dispatchOnChange(lastFloorSpeed, speed, ::SetFloorSpeed) { lastFloorSpeed = it }
        }
    }
}
