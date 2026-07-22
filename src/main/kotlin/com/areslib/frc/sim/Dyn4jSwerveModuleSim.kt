package com.areslib.frc.sim

import org.dyn4j.dynamics.Body
import org.dyn4j.geometry.Vector2
import com.areslib.state.RobotState

class Dyn4jSwerveModuleSim {
    private val forceVector = Vector2()

    fun update(state: RobotState, robotBody: Body) {
        val kpLinear = 50.0
        val kpAngular = 20.0
        val forceX = (state.drive.xVelocityMetersPerSecond - robotBody.linearVelocity.x) * kpLinear
        val forceY = (state.drive.yVelocityMetersPerSecond - robotBody.linearVelocity.y) * kpLinear
        val torque = (state.drive.angularVelocityRadiansPerSecond - robotBody.angularVelocity) * kpAngular

        robotBody.isAtRest = false
        forceVector.set(forceX, forceY)
        robotBody.applyForce(forceVector)
        robotBody.applyTorque(torque)
    }
}
