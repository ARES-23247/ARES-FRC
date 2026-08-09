package com.areslib.frc

import com.areslib.pathing.Path
import com.areslib.pathing.PathPlannerParser
import java.io.BufferedReader
import java.io.File
import java.io.InputStreamReader
/**
 * Documentation for PathLoader
 */

object PathLoader {

    /**
     * Loads and parses a PathPlanner path JSON.
     *
     * Resolution order:
     * 1. Physical filesystem via [edu.wpi.first.wpilibj.Filesystem.getDeployDirectory] (RoboRIO).
     * 2. Classpath resource lookup (JUnit / desktop simulation fallback).
     */
    fun loadPath(pathName: String): Path {
        val relativePath = "pathplanner/paths/$pathName.path"

        // Try physical deploy directory first (RoboRIO)
        val deployFile = try {
            File(edu.wpi.first.wpilibj.Filesystem.getDeployDirectory(), relativePath)
        } catch (_: Throwable) {
            null
        }

        val jsonString = when {
            deployFile != null && deployFile.exists() -> deployFile.readText(Charsets.UTF_8)
            else -> {
                val resourcePath = "/deploy/$relativePath"
                val inputStream = javaClass.getResourceAsStream(resourcePath)
                    ?: throw IllegalArgumentException(
                        "Could not find path '$pathName' in deploy directory or classpath ($resourcePath)"
                    )
                BufferedReader(InputStreamReader(inputStream, Charsets.UTF_8)).use { it.readText() }
            }
        }

        return PathPlannerParser.parsePath(jsonString)
    }
}
