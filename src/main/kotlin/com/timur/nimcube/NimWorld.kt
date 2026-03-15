package com.timur.nimcube

import org.bukkit.Bukkit
import org.bukkit.World
import org.bukkit.scheduler.BukkitTask
import org.joml.Vector3f
import kotlin.math.round

class NimWorld(val plugin: Nimcube, val bukkitWorld: World, val dt: Float, val acceleration: Vector3f) {
    val nim = plugin.nim
    val worldIndex = nim.createWorld(dt, acceleration)
    val cuboids = arrayListOf<Cuboid>()
    var task: BukkitTask? = null

    fun init() {
        task = Bukkit.getScheduler().runTaskTimer(plugin, Runnable { update() }, 1, 1)
    }

    fun update() {
        repeat(round(0.05 / dt).toInt()) {
            nim.tickWorld(worldIndex)
        }
        cuboids.forEach { it.update() }
    }

    fun deinit() {
        cuboids.forEach { it.deinit() }
        task?.cancel()
    }
}