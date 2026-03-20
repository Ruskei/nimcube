package com.timur.nimcube

import com.timur.nimcube.command.*
import org.bukkit.World
import org.bukkit.plugin.java.JavaPlugin
import org.joml.Vector3f
import java.util.concurrent.ConcurrentHashMap

class Nimcube : JavaPlugin() {
    val nim = Nim(this)
    val nimWorlds = ConcurrentHashMap<World, NimWorld>()

    val Δt = 0.05f / 3f
    val acceleration = Vector3f(0f, 0f, 0f)

    override fun onEnable() {
        GreedyMeshTestCommand(this).init()
        CuboidCommand(this).init()
        PortalCommand(this).init()
        GridCuboidsCommand(this).init()
        ClearCommand(this).init()
        ClearPortalsCommand(this).init()
        FreezeCommand(this).init()
        StepCommand(this).init()
    }

    override fun onDisable() {
        nim.deinit()

        nimWorlds.values.forEach { it.deinit() }
        nimWorlds.clear()
    }
}
