package com.areslib.frc.generatedruntime

import com.areslib.action.RobotAction
import com.areslib.frc.generated.GeneratedAresProjectControlTaskSink
import com.areslib.frc.input.FrcInputFrameAdapter
import com.areslib.input.ControllerBindingRuntime
import com.areslib.input.InputFrame
import com.areslib.sequencer.Task
import com.areslib.sequencer.TaskExecutor
import com.areslib.state.RobotState
import com.areslib.util.RobotClock
import edu.wpi.first.wpilibj.GenericHID

/** One generated controller slot paired with its canonical FRC HID adapter. */
class FrcControllerBindingSlot(
    val slotId: String,
    val port: Int,
    val runtime: ControllerBindingRuntime,
    val frame: InputFrame = InputFrame(),
    val adapter: FrcInputFrameAdapter = FrcInputFrameAdapter(GenericHID(port))
) {
    init {
        require(slotId.isNotBlank()) { "Controller slot ID must not be blank" }
        require(port >= 0) { "Controller port must be non-negative" }
    }
}

/**
 * FRC platform boundary for generated control schemes.
 *
 * [FrcInputFrameAdapter] owns raw-HID normalization, including vendor buttons and POV virtual
 * buttons 120–123. The host samples every slot, evaluates generated bindings, then advances their
 * action/routine schedulers. Normal polling reuses all frames and adapters.
 */
class FrcControllerBindingHost(
    slots: Collection<FrcControllerBindingSlot>,
    private val afterBindingsUpdate: () -> Unit = {},
    private val afterBindingsCancel: () -> Unit = {}
) {
    private val activeSlots = slots.toTypedArray()

    fun update() {
        var index = 0
        while (index < activeSlots.size) {
            val slot = activeSlots[index]
            slot.adapter.sampleInto(slot.frame)
            slot.runtime.update(slot.frame)
            index++
        }
        afterBindingsUpdate()
    }

    fun cancel() {
        var index = 0
        while (index < activeSlots.size) {
            activeSlots[index].runtime.cancel()
            index++
        }
        afterBindingsCancel()
    }
}

/** Executes fresh direct-action tasks submitted by generated button and analog bindings. */
class FrcGeneratedControlTaskScheduler(
    private val stateProvider: () -> RobotState,
    private val dispatch: (RobotAction) -> Unit
) : GeneratedAresProjectControlTaskSink {
    private val executor = TaskExecutor()

    override fun submit(bindingId: String, task: Task) {
        require(bindingId.isNotBlank()) { "Generated binding ID must not be blank" }
        executor.addTask(task)
    }

    fun update() {
        executor.update(stateProvider(), RobotClock.currentTimeMillis()).forEach(dispatch)
    }

    fun cancel() {
        executor.cancelAll(stateProvider()).forEach(dispatch)
    }
}

/** Deterministic project convention for generated controller slots. */
object FrcGeneratedControllerPorts {
    const val DRIVER_PORT: Int = 0
    const val OPERATOR_PORT: Int = 1

    fun resolve(slotId: String): Int = when (slotId.trim().lowercase()) {
        "driver" -> DRIVER_PORT
        "operator" -> OPERATOR_PORT
        else -> error(
            "Generated FRC controller slot '$slotId' is unsupported; use 'driver' or 'operator'"
        )
    }
}

/** Stable default until the project schema grows an explicit preferred scheme field. */
fun selectDefaultGeneratedControlScheme(knownSchemeIds: Set<String>): String? =
    if ("competition" in knownSchemeIds) "competition" else knownSchemeIds.minOrNull()
