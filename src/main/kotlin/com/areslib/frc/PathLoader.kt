package com.areslib.frc

import com.areslib.pathing.Path
import com.areslib.pathing.PathPlannerParser
import java.io.BufferedReader
import java.io.InputStreamReader

object PathLoader {

    /**
     * Loads and parses a PathPlanner path JSON from classpath resources.
     */
    fun loadPath(pathName: String): Path {
        val resourcePath = "/deploy/pathplanner/paths/$pathName.path"
        val inputStream = javaClass.getResourceAsStream(resourcePath)
            ?: throw IllegalArgumentException("Could not find path resource at $resourcePath")
        
        val jsonString = BufferedReader(InputStreamReader(inputStream, Charsets.UTF_8)).use { reader ->
            reader.readText()
        }
        
        return PathPlannerParser.parsePath(jsonString)
    }
}
