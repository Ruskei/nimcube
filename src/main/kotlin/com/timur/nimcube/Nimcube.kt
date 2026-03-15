package com.timur.nimcube

import org.bukkit.plugin.java.JavaPlugin

class Nimcube : JavaPlugin() {
    val nim = Nim(this)

    override fun onEnable() {
        GreedyMeshTestCommand(this).init()
    }

    override fun onDisable() {
        nim.deinit()
    }
}