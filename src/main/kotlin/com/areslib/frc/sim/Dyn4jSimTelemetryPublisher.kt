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
    /**
     * Documentation for publishVisualization
     */

    fun publishVisualization(
        state: RobotState,
        telemetry: ITelemetry,
        intakeAngleDegrees: Double,
        simCowlAngle: Double,
        flywheelRotationAngle: Double,
        balls: List<Body>,
        flyingBalls: List<FlyingBall>
    ) {
        /**
         * Documentation for robotX
         */
        val robotX = state.drive.odometryX
        /**
         * Documentation for robotY
         */
        val robotY = state.drive.odometryY
        /**
         * Documentation for robotHeading
         */
        val robotHeading = state.drive.odometryHeading
        /**
         * Documentation for halfHeading
         */

        val halfHeading = robotHeading / 2.0
        /**
         * Documentation for robotQW
         */
        val robotQW = Math.cos(halfHeading)
        /**
         * Documentation for robotQZ
         */
        val robotQZ = Math.sin(halfHeading)

        fun publishSubsystemPose(key: String, dx: Double, dz: Double, pitchRad: Double) {
            val halfPitch = pitchRad / 2.0
            val pCos = Math.cos(halfPitch)
            val pSin = Math.sin(halfPitch)
            telemetry.putDoubleArray(key, doubleArrayOf(
                robotX + dx * Math.cos(robotHeading),
                robotY + dx * Math.sin(robotHeading),
                dz,
                robotQW * pCos, -robotQZ * pSin, robotQW * pSin, robotQZ * pCos
            ))
        }

        // ── Intake 3D Pose ──
        publishSubsystemPose("Robot/Superstructure/3D/Intake", 0.35, 0.2, Math.toRadians(intakeAngleDegrees))

        // ── Cowl 3D Pose ──
        publishSubsystemPose("Robot/Superstructure/3D/Cowl", -0.2, 0.6, Math.toRadians(simCowlAngle))

        // ── Flywheel 3D Pose ──
        publishSubsystemPose("Robot/Superstructure/3D/Flywheel", -0.1, 0.6, flywheelRotationAngle)

        // ── Fuel 3D Poses ──
        /**
         * Documentation for totalBallsCount
         */
        val totalBallsCount = balls.size + flyingBalls.size
        if (gamePieceData.size < totalBallsCount * 7) {
            gamePieceData = DoubleArray(totalBallsCount * 7 * 2)
        }
        for (i in balls.indices) {
            /**
             * Documentation for idx
             */
            val idx = i * 7
            gamePieceData[idx] = balls[i].transform.translationX
            gamePieceData[idx + 1] = balls[i].transform.translationY
            gamePieceData[idx + 2] = 0.0635
            /**
             * Documentation for theta
             */
            val theta = balls[i].transform.rotationAngle
            gamePieceData[idx + 3] = kotlin.math.cos(theta / 2.0)
            gamePieceData[idx + 4] = 0.0
            gamePieceData[idx + 5] = 0.0
            gamePieceData[idx + 6] = kotlin.math.sin(theta / 2.0)
        }
        /**
         * Documentation for groundOffset
         */
        val groundOffset = balls.size * 7
        for (i in flyingBalls.indices) {
            /**
             * Documentation for fb
             */
            val fb = flyingBalls[i]
            /**
             * Documentation for idx
             */
            val idx = groundOffset + i * 7
            gamePieceData[idx] = fb.x
            gamePieceData[idx + 1] = fb.y
            gamePieceData[idx + 2] = fb.z
            gamePieceData[idx + 3] = 1.0 // qw
            gamePieceData[idx + 4] = 0.0 // qx
            gamePieceData[idx + 5] = 0.0 // qy
            gamePieceData[idx + 6] = 0.0 // qz
        }
        telemetry.putDoubleArray("Robot/FuelPoses", gamePieceData.copyOfRange(0, totalBallsCount * 7))
    }
}
