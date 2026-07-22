package com.areslib.frc.sim

import org.dyn4j.dynamics.Body
import org.dyn4j.geometry.Vector2
import com.areslib.state.RobotState
/**
 * Documentation for Dyn4jSwerveModuleSim
 */

class Dyn4jSwerveModuleSim {
    private val forceVector = Vector2()
    /**
     * Documentation for update
     */

    fun update(state: RobotState, robotBody: Body) {
        /**
         * Documentation for kpLinear
         */
        val kpLinear = 50.0
        /**
         * Documentation for kpAngular
         */
        val kpAngular = 20.0
        /**
         * Documentation for forceX
         */
        val forceX = (state.drive.xVelocityMetersPerSecond - robotBody.linearVelocity.x) * kpLinear
        /**
         * Documentation for forceY
         */
        val forceY = (state.drive.yVelocityMetersPerSecond - robotBody.linearVelocity.y) * kpLinear
        /**
         * Documentation for torque
         */
        val torque = (state.drive.angularVelocityRadiansPerSecond - robotBody.angularVelocity) * kpAngular

        robotBody.isAtRest = false
        forceVector.set(forceX, forceY)
        robotBody.applyForce(forceVector)
        robotBody.applyTorque(torque)
    }
}
