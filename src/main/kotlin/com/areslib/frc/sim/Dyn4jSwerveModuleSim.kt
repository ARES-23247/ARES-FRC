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
        val kpLinear = 50.0
        /**
         * Documentation for kpAngular
         */
        val kpAngular = 20.0
        
        val heading = robotBody.transform.rotationAngle
        val targetVx = state.drive.xVelocityMetersPerSecond
        val targetVy = state.drive.yVelocityMetersPerSecond
        val worldVx = targetVx * kotlin.math.cos(heading) - targetVy * kotlin.math.sin(heading)
        val worldVy = targetVx * kotlin.math.sin(heading) + targetVy * kotlin.math.cos(heading)
        
        /**
         * Documentation for forceX
         */
        val forceX = (worldVx - robotBody.linearVelocity.x) * kpLinear
        /**
         * Documentation for forceY
         */
        val forceY = (worldVy - robotBody.linearVelocity.y) * kpLinear
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
