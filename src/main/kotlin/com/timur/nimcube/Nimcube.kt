package com.timur.nimcube

import com.timur.nimcube.command.CuboidCommand
import com.timur.nimcube.command.ClearCommand
import com.timur.nimcube.command.GreedyMeshTestCommand
import org.bukkit.World
import org.bukkit.plugin.java.JavaPlugin
import org.joml.Vector3f
import java.util.concurrent.ConcurrentHashMap

class Nimcube : JavaPlugin() {
    val nim = Nim(this)
    val nimWorlds = ConcurrentHashMap<World, NimWorld>()
    
    val Δt = 0.01f
    val acceleration = Vector3f(0f, -1f, 0f)

    override fun onEnable() {
        GreedyMeshTestCommand(this).init()
        CuboidCommand(this).init()
        ClearCommand(this).init()
    }

    override fun onDisable() {
        nim.deinit()

        nimWorlds.values.forEach { it.deinit() }
        nimWorlds.clear()
    }
}
