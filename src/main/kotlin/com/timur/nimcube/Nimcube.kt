package com.timur.nimcube

import com.timur.nimcube.command.CuboidCommand
import com.timur.nimcube.command.GreedyMeshTestCommand
import org.bukkit.World
import org.bukkit.plugin.java.JavaPlugin
import java.util.concurrent.ConcurrentHashMap

class Nimcube : JavaPlugin() {
    val nim = Nim(this)
    val physicsWorlds = ConcurrentHashMap<World, Nim.WorldIndex>()

    override fun onEnable() {
        GreedyMeshTestCommand(this).init()
        CuboidCommand(this).init()
    }

    override fun onDisable() {
        nim.deinit()
    }
}