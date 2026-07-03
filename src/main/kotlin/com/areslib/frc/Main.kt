package com.areslib.frc

import edu.wpi.first.wpilibj.RobotBase

/**
 * Main initialization class for the FRC application.
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what you are doing, do not modify this file except to
 * change the parameter class to the startRobot call.
 */
object Main {
    @JvmStatic
    fun main(args: Array<String>) {
        RobotBase.startRobot { ARESRobot() }
    }
}
