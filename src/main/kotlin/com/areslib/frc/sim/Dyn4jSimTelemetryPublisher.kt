package com.areslib.frc.sim

import org.dyn4j.dynamics.Body
import com.areslib.state.RobotState
import com.areslib.telemetry.ITelemetry
import com.areslib.frc.FlyingBall

class Dyn4jSimTelemetryPublisher {
    private var gamePieceData = DoubleArray(100 * 7)

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

        // ── Intake 3D Pose ──
        val intakeAngleRad = Math.toRadians(intakeAngleDegrees)
        val halfIntake = intakeAngleRad / 2.0
        val intCosY = Math.cos(halfIntake)
        val intSinY = Math.sin(halfIntake)
        telemetry.putDoubleArray("Robot/Superstructure/3D/Intake", doubleArrayOf(
            robotX + 0.35 * Math.cos(robotHeading),
            robotY + 0.35 * Math.sin(robotHeading),
            0.2,
            robotQW * intCosY, -robotQZ * intSinY, robotQW * intSinY, robotQZ * intCosY
        ))

        // ── Cowl 3D Pose ──
        val cowlAngleRad = Math.toRadians(simCowlAngle)
        val halfCowl = cowlAngleRad / 2.0
        val cowlCosY = Math.cos(halfCowl)
        val cowlSinY = Math.sin(halfCowl)
        telemetry.putDoubleArray("Robot/Superstructure/3D/Cowl", doubleArrayOf(
            robotX - 0.2 * Math.cos(robotHeading),
            robotY - 0.2 * Math.sin(robotHeading),
            0.6,
            robotQW * cowlCosY, -robotQZ * cowlSinY, robotQW * cowlSinY, robotQZ * cowlCosY
        ))

        // ── Flywheel 3D Pose ──
        val halfFlywheel = flywheelRotationAngle / 2.0
        val flyCosY = Math.cos(halfFlywheel)
        val flySinY = Math.sin(halfFlywheel)
        telemetry.putDoubleArray("Robot/Superstructure/3D/Flywheel", doubleArrayOf(
            robotX - 0.1 * Math.cos(robotHeading),
            robotY - 0.1 * Math.sin(robotHeading),
            0.6,
            robotQW * flyCosY, -robotQZ * flySinY, robotQW * flySinY, robotQZ * flyCosY
        ))

        // ── Fuel 3D Poses ──
        val totalBallsCount = balls.size + flyingBalls.size
        if (gamePieceData.size < totalBallsCount * 7) {
            gamePieceData = DoubleArray(totalBallsCount * 7 * 2)
        }
        for (i in balls.indices) {
            val idx = i * 7
            gamePieceData[idx] = balls[i].transform.translationX
            gamePieceData[idx + 1] = balls[i].transform.translationY
            gamePieceData[idx + 2] = 0.0635
            val theta = balls[i].transform.rotationAngle
            gamePieceData[idx + 3] = kotlin.math.cos(theta / 2.0)
            gamePieceData[idx + 4] = 0.0
            gamePieceData[idx + 5] = 0.0
            gamePieceData[idx + 6] = kotlin.math.sin(theta / 2.0)
        }
        val groundOffset = balls.size * 7
        for (i in flyingBalls.indices) {
            val fb = flyingBalls[i]
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
