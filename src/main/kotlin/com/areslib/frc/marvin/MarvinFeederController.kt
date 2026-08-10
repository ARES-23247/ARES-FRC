package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.action.RobotAction

class MarvinFeederController(store: Store) : MarvinControllerBase(store) {
    /**
     * Documentation for transferActive
     */

    val transferActive: Boolean
        get() = store.state.superstructure.marvin.transferActive
    /**
     * Documentation for shoot
     */

    fun shoot() {
        dispatchOnChange(store.state.superstructure.marvin.transferActive, true, ::SetTransferActive) {}
        dispatchOnChange(store.state.superstructure.marvin.feeder.targetVelocityRps, MarvinConfig.FEEDER_SHOOT_SPEED_RPS, ::SetFeederSpeed) {}
    }
    /**
     * Documentation for stop
     */

    fun stop() {
        dispatchOnChange(store.state.superstructure.marvin.transferActive, false, ::SetTransferActive) {}
        dispatchOnChange(store.state.superstructure.marvin.feeder.targetVelocityRps, 0.0, ::SetFeederSpeed) {}
        dispatchOnChange(store.state.superstructure.marvin.floor.targetVelocityRps, 0.0, ::SetFloorSpeed) {}
    }
    /**
     * Documentation for updateFeeders
     */

    fun updateFeeders(rpmAligned: Boolean, headingAligned: Boolean, runFloorRollers: Boolean = false) {
        /**
         * Documentation for speed
         */
        val speed = if ((headingAligned && rpmAligned) || transferActive == true) 10.0 else 0.0
        dispatchOnChange(store.state.superstructure.marvin.feeder.targetVelocityRps, speed, ::SetFeederSpeed) {}
        
        if (runFloorRollers) {
            dispatchOnChange(store.state.superstructure.marvin.floor.targetVelocityRps, speed, ::SetFloorSpeed) {}
        } else {
            dispatchOnChange(store.state.superstructure.marvin.floor.targetVelocityRps, 0.0, ::SetFloorSpeed) {}
        }
    }
}
