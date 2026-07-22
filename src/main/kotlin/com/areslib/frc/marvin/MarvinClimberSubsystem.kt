package com.areslib.frc.marvin

import com.areslib.Store

class MarvinClimberSubsystem(store: Store) : MarvinControllerBase(store) {
    private var lastTargetExtension = Double.NaN
    private var lastTargetVoltage = Double.NaN

    /**
     * Documentation for extensionMeters
     */
    val extensionMeters: Double
        get() = store.state.superstructure.marvin.climber.extensionMeters

    /**
     * Documentation for targetExtensionMeters
     */
    val targetExtensionMeters: Double
        get() = store.state.superstructure.marvin.climber.targetExtensionMeters

    /**
     * Documentation for setTargetExtension
     */
    fun setTargetExtension(meters: Double) {
        dispatchOnChange(lastTargetExtension, meters, ::SetClimberExtension) { lastTargetExtension = it }
    }

    /**
     * Documentation for setVoltage
     */
    fun setVoltage(volts: Double) {
        dispatchOnChange(lastTargetVoltage, volts, ::SetClimberVoltage) { lastTargetVoltage = it }
    }
}
