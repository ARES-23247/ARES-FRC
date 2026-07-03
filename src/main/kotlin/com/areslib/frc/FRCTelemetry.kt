package com.areslib.frc

import com.areslib.telemetry.ITelemetry
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.networktables.NetworkTableInstance

/**
 * FRC-specific implementation of the telemetry interface.
 * Routes all variables to WPILib's SmartDashboard (NetworkTables 4).
 * By starting DataLogManager, all NT4 changes are automatically logged deterministically to a .wpilog file.
 */
class FRCTelemetry : ITelemetry {

    init {
        // Starts the deterministic logger (logs all NT4 values, joystick inputs, and console output to a .wpilog)
        DataLogManager.start()
        
        // Explicitly start NT4 Server for AdvantageScope to connect to
        NetworkTableInstance.getDefault().startServer()
        
        // Log custom strings or events if necessary
        DataLogManager.log("ARESLib: FRC Telemetry Initialized")
    }

    override fun putNumber(key: String, value: Double) {
        SmartDashboard.putNumber(key, value)
    }

    override fun putBoolean(key: String, value: Boolean) {
        SmartDashboard.putBoolean(key, value)
    }

    override fun putString(key: String, value: String) {
        SmartDashboard.putString(key, value)
    }

    override fun putDoubleArray(key: String, value: DoubleArray) {
        SmartDashboard.putNumberArray(key, value)
    }

    override fun getNumber(key: String, defaultValue: Double): Double {
        return SmartDashboard.getNumber(key, defaultValue)
    }

    override fun getBoolean(key: String, defaultValue: Boolean): Boolean {
        return SmartDashboard.getBoolean(key, defaultValue)
    }

    override fun getString(key: String, defaultValue: String): String {
        return SmartDashboard.getString(key, defaultValue)
    }
}
