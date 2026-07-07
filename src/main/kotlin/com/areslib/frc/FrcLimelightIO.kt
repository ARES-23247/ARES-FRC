package com.areslib.frc

import com.areslib.hardware.vision.VisionIO
import com.areslib.hardware.vision.VisionIOInputs
import com.areslib.state.VisionMeasurement
import com.areslib.math.Pose3d
import com.areslib.math.Translation3d
import com.areslib.math.Rotation3d
import edu.wpi.first.networktables.NetworkTableInstance

/**
 * FRC-specific implementation of the VisionIO abstraction for Limelight.
 * Reads 3D field-relative robot pose data directly from NetworkTables (NT4).
 */
class FrcLimelightIO(
    val tableName: String = "limelight",
    override val cameraPoses: List<Pose3d> = listOf(Pose3d(Translation3d(0.18, 0.0, 0.0), Rotation3d(0.0, 0.0, 0.0))),
    val defaultPipeline: Int = 0,
    val imuMode: Int = 4 // 4 = INTERNAL_EXTERNAL_ASSIST (recommended for LL3G/LL4), 0 = EXTERNAL_ONLY
) : VisionIO {

    private val table = NetworkTableInstance.getDefault().getTable(tableName)
    private val botposeSub = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(DoubleArray(0))
    private val botposeMt2Sub = table.getDoubleArrayTopic("botpose_wpiblue_mt2").subscribe(DoubleArray(0))
    private val tvSub = table.getIntegerTopic("tv").subscribe(0)
    
    private val orientationPub = table.getDoubleArrayTopic("orientation_megatag2").publish()

    private var lastLinearVelocityMps = 0.0
    private var lastYawRateDegPerSec = 0.0
    
    // Pre-allocated buffers to prevent GC
    private val scratchOrientation = DoubleArray(6)
    private val scratchMeasurements = ArrayList<VisionMeasurement>(1)

    init {
        // Enforce match-ready settings to NetworkTables on startup
        try {
            table.getIntegerTopic("pipeline").publish().set(defaultPipeline.toLong())
            table.getIntegerTopic("ledMode").publish().set(1L) // 1 = Force Off
            table.getIntegerTopic("stream").publish().set(0L)  // 0 = Standard Stream
            table.getIntegerTopic("imuMode").publish().set(imuMode.toLong())
        } catch (e: Exception) {
            System.err.println("FrcLimelightIO: Failed to write startup configuration: ${e.message}")
        }
    }

    override fun setOrientation(
        yawDegrees: Double, yawRateDegPerSec: Double,
        pitchDegrees: Double, pitchRateDegPerSec: Double,
        rollDegrees: Double, rollRateDegPerSec: Double,
        linearVelocityMps: Double
    ) {
        lastLinearVelocityMps = linearVelocityMps
        lastYawRateDegPerSec = yawRateDegPerSec
        
        scratchOrientation[0] = yawDegrees
        scratchOrientation[1] = yawRateDegPerSec
        scratchOrientation[2] = pitchDegrees
        scratchOrientation[3] = pitchRateDegPerSec
        scratchOrientation[4] = rollDegrees
        scratchOrientation[5] = rollRateDegPerSec
        
        orientationPub.set(scratchOrientation)
    }

    override fun updateInputs(inputs: VisionIOInputs) {
        inputs.cameraPoses = cameraPoses
        
        val tv = tvSub.get()
        val isStatic = lastLinearVelocityMps < 0.15 && Math.abs(lastYawRateDegPerSec) < 10.0
        
        // Select topic based on robot motion state (Hybrid MegaTag strategy)
        // - MegaTag1 (botpose_wpiblue) when static to allow EKF to correct yaw drift.
        // - MegaTag2 (botpose_wpiblue_mt2) when moving to prevent pose-flipping.
        val botpose = if (isStatic) {
            val mt1 = botposeSub.get()
            if (mt1.isNotEmpty()) mt1 else botposeMt2Sub.get()
        } else {
            val mt2 = botposeMt2Sub.get()
            if (mt2.isNotEmpty()) mt2 else botposeSub.get()
        }
        
        inputs.isConnected = tv == 1L && botpose.isNotEmpty()
        
        if (inputs.isConnected && botpose.size >= 6) {
            val x = botpose[0]
            val y = botpose[1]
            val z = botpose[2]
            val roll = Math.toRadians(botpose[3])
            val pitch = Math.toRadians(botpose[4])
            val yaw = Math.toRadians(botpose[5])
            
            // Limelight latency (ms) is typically index 6.
            val latencyMs = if (botpose.size > 6) botpose[6] else 0.0
            val timestampMs = com.areslib.util.RobotClock.currentTimeMillis() - latencyMs.toLong()
            
            // Limelight's botpose_wpiblue array does not contain single-tag ambiguity.
            // Index 10 represents Average Target Area (percent), which is typically > 0.15% 
            // for good close-up tag readings, causing false outlier rejects. We set ambiguity 
            // to a stable constant (0.02) as multitag pose estimations are extremely stable.
            val ambiguity = 0.02
            
            val pose = Pose3d(
                translation = Translation3d(x, y, z),
                rotation = Rotation3d(roll, pitch, yaw)
            )
            
            scratchMeasurements.clear()
            scratchMeasurements.add(VisionMeasurement(
                timestampMs = timestampMs,
                targetPose = pose,
                tagId = -1,
                ambiguity = ambiguity
            ))
            
            inputs.measurements = scratchMeasurements.toList()
        } else {
            inputs.measurements = emptyList()
        }
    }
}
