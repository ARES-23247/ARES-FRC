package com.areslib.frc.hardware

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX

fun Iterable<TalonFX>.applyConfig(block: TalonFXConfiguration.() -> Unit) {
    val config = TalonFXConfiguration()
    config.block()
    for (motor in this) {
        motor.configurator.apply(config)
    }
}

fun setUpdateFrequencies(hz: Double, vararg signals: BaseStatusSignal) {
    for (signal in signals) {
        signal.setUpdateFrequency(hz)
    }
}
