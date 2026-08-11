package com.areslib.frc.hardware

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj.DriverStation

/** Exposes whether one-time mechanism configuration completed successfully. */
interface FrcMechanismConfigurationStatus {
    val configurationValid: Boolean
}

/**
 * Adds mechanism context to ARESLib's checked Talon configuration result.
 */
internal fun Iterable<TalonFX>.applyMechanismConfigChecked(
    mechanismName: String,
    block: TalonFXConfiguration.() -> Unit
): Boolean {
    val applied = applyConfigChecked(block = block)
    if (!applied) reportConfigurationFailure("$mechanismName configuration failed")
    return applied
}

internal fun reportConfigurationFailure(message: String) {
    runCatching { DriverStation.reportError("ARES: $message; mechanism outputs inhibited", false) }
        .onFailure { System.err.println("ARES: $message; mechanism outputs inhibited") }
}
