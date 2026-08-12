package com.areslib.frc.sim

import org.dyn4j.dynamics.Body
import org.dyn4j.geometry.Vector2
import com.areslib.state.RobotState

/**
 * Applies proportional force and torque so the Dyn4j chassis tracks Redux drive setpoints.
 *
 * Redux linear velocities are interpreted according to `DriveState.isFieldCentric`; field-frame
 * commands are used directly and robot-frame commands are rotated once into Dyn4j's blue-origin
 * world. Angular velocity is CCW-positive radians per second. [forceVector] is reused.
 */
class Dyn4jSwerveModuleSim {
    private val forceVector = Vector2()

    /** Applies one tick's tracking effort without advancing the physics world. */
    fun update(state: RobotState, robotBody: Body) {
        val kpLinear = 50.0
        val kpAngular = 20.0
        
        val heading = robotBody.transform.rotationAngle
        val targetVx = state.drive.xVelocityMetersPerSecond
        val targetVy = state.drive.yVelocityMetersPerSecond
        val worldVx: Double
        val worldVy: Double
        if (state.drive.isFieldCentric) {
            worldVx = targetVx
            worldVy = targetVy
        } else {
            worldVx = targetVx * kotlin.math.cos(heading) - targetVy * kotlin.math.sin(heading)
            worldVy = targetVx * kotlin.math.sin(heading) + targetVy * kotlin.math.cos(heading)
        }
        
        val forceX = (worldVx - robotBody.linearVelocity.x) * kpLinear
        val forceY = (worldVy - robotBody.linearVelocity.y) * kpLinear
        val torque = (state.drive.angularVelocityRadiansPerSecond - robotBody.angularVelocity) * kpAngular

        robotBody.isAtRest = false
        forceVector.set(forceX, forceY)
        robotBody.applyForce(forceVector)
        robotBody.applyTorque(torque)
    }
}
