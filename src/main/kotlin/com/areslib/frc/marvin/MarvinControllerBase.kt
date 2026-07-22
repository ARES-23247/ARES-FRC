package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.action.RobotAction

abstract class MarvinControllerBase(protected val store: Store) {
    protected inline fun <T> dispatchOnChange(
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
}
