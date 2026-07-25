package com.areslib.frc.sim

import org.dyn4j.dynamics.Body
import com.areslib.state.RobotState
import com.areslib.telemetry.ITelemetry
import com.areslib.frc.FlyingBall
/**
 * Documentation for Dyn4jSimTelemetryPublisher
 */

class Dyn4jSimTelemetryPublisher {
    private var gamePieceData = DoubleArray(100 * 7)
    private var activeFuelData = DoubleArray(100 * 7)
    private val subsystemPoseBuf = DoubleArray(7)

    fun publishVisualization(
        state: RobotState,
        telemetry: ITelemetry,
        intakeAngleDegrees: Double,
        simCowlAngle: Double,
        flywheelRotationAngle: Double,
        balls: List<Body>,
        flyingBalls: List<FlyingBall>
    ) {
        val robotX = state.drive.odometryX
        val robotY = state.drive.odometryY
        val robotHeading = state.drive.odometryHeading

        val halfHeading = robotHeading / 2.0
        val robotQW = Math.cos(halfHeading)
        val robotQZ = Math.sin(halfHeading)

        fun publishSubsystemPose(key: String, dx: Double, dz: Double, pitchRad: Double) {
            val halfPitch = pitchRad / 2.0
            val pCos = Math.cos(halfPitch)
            val pSin = Math.sin(halfPitch)
            subsystemPoseBuf[0] = robotX + dx * Math.cos(robotHeading)
            subsystemPoseBuf[1] = robotY + dx * Math.sin(robotHeading)
            subsystemPoseBuf[2] = dz
            subsystemPoseBuf[3] = robotQW * pCos
            subsystemPoseBuf[4] = -robotQZ * pSin
            subsystemPoseBuf[5] = robotQW * pSin
            subsystemPoseBuf[6] = robotQZ * pCos
            telemetry.putDoubleArray(key, subsystemPoseBuf)
        }

        // ── Intake 3D Pose ──
        publishSubsystemPose("Robot/Superstructure/3D/Intake", 0.35, 0.2, Math.toRadians(intakeAngleDegrees))

        // ── Cowl 3D Pose ──
        publishSubsystemPose("Robot/Superstructure/3D/Cowl", -0.2, 0.6, Math.toRadians(simCowlAngle))

        // ── Flywheel 3D Pose ──
        publishSubsystemPose("Robot/Superstructure/3D/Flywheel", -0.1, 0.6, flywheelRotationAngle)

        // ── Fuel 3D Poses ──
        val totalBallsCount = balls.size + flyingBalls.size
        val neededSize = totalBallsCount * 7
        if (neededSize > activeFuelData.size) {
            activeFuelData = DoubleArray(neededSize + 70)
        }
        for (i in balls.indices) {
            val idx = i * 7
            activeFuelData[idx] = balls[i].transform.translationX
            activeFuelData[idx + 1] = balls[i].transform.translationY
            activeFuelData[idx + 2] = 0.0635
            val theta = balls[i].transform.rotationAngle
            activeFuelData[idx + 3] = kotlin.math.cos(theta / 2.0)
            activeFuelData[idx + 4] = 0.0
            activeFuelData[idx + 5] = 0.0
            activeFuelData[idx + 6] = kotlin.math.sin(theta / 2.0)
        }
        val groundOffset = balls.size * 7
        for (i in flyingBalls.indices) {
            val fb = flyingBalls[i]
            val idx = groundOffset + i * 7
            activeFuelData[idx] = fb.x
            activeFuelData[idx + 1] = fb.y
            activeFuelData[idx + 2] = fb.z
            activeFuelData[idx + 3] = 1.0 // qw
            activeFuelData[idx + 4] = 0.0 // qx
            activeFuelData[idx + 5] = 0.0 // qy
            activeFuelData[idx + 6] = 0.0 // qz
        }
        telemetry.putDoubleArray("Robot/FuelPoses", activeFuelData)
    }
}
