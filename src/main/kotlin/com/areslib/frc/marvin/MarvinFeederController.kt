package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.action.RobotAction

class MarvinFeederController(store: Store) : MarvinControllerBase(store) {
    private var lastFeederSpeed = Double.NaN
    private var lastFloorSpeed = Double.NaN
    private var lastTransferActive: Boolean? = null
    /**
     * Documentation for transferActive
     */

    val transferActive: Boolean
        get() = store.state.superstructure.marvin.transferActive
    /**
     * Documentation for shoot
     */

    fun shoot() {
        dispatchOnChange(lastTransferActive, true, ::SetTransferActive) { lastTransferActive = it }
    }
    /**
     * Documentation for stop
     */

    fun stop() {
        dispatchOnChange(lastTransferActive, false, ::SetTransferActive) { lastTransferActive = it }
    }
    /**
     * Documentation for updateFeeders
     */

    fun updateFeeders(rpmAligned: Boolean, headingAligned: Boolean, runFloorRollers: Boolean = false) {
        /**
         * Documentation for speed
         */
        val speed = if (headingAligned && rpmAligned) 10.0 else 0.0
        dispatchOnChange(lastFeederSpeed, speed, ::SetFeederSpeed) { lastFeederSpeed = it }
        
        if (runFloorRollers) {
            dispatchOnChange(lastFloorSpeed, speed, ::SetFloorSpeed) { lastFloorSpeed = it }
        }
    }
}
