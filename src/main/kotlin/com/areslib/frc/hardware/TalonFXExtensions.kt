package com.areslib.frc.hardware

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
/**
 * Documentation for Iterable
 */

fun Iterable<TalonFX>.applyConfig(block: TalonFXConfiguration.() -> Unit) {
    /**
     * Documentation for config
     */
    val config = TalonFXConfiguration()
    config.block()
    for (motor in this) {
        motor.configurator.apply(config)
    }
}
/**
 * Documentation for setUpdateFrequencies
 */

fun setUpdateFrequencies(hz: Double, vararg signals: BaseStatusSignal) {
    for (signal in signals) {
        signal.setUpdateFrequency(hz)
    }
}
