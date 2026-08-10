package com.areslib.frc.sim.io

import com.areslib.frc.hardware.FeederIO
import com.areslib.frc.Dyn4jSimulation

class SimulatedFeederIO(
    private val sim: Dyn4jSimulation,
    private val detectorConfigured: Boolean = false
) : FeederIO {
    override fun setAppliedVoltage(volts: Double) {
        sim.simFeederVoltage = volts.coerceIn(-12.0, 12.0)
    }
    override val isBeamBroken: Boolean get() = detectorConfigured && sim.simFeederPieceDetected
    override val pieceDetectionValid: Boolean get() = detectorConfigured
    override val currentAmps: Double get() = Math.abs(sim.simFeederVoltage) * 0.1
}
