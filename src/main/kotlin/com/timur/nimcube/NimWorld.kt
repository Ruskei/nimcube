package com.timur.nimcube

import org.bukkit.Bukkit
import org.bukkit.World
import org.bukkit.scheduler.BukkitTask
import org.joml.Vector3f
import java.lang.foreign.Arena
import java.util.concurrent.ConcurrentLinkedDeque
import kotlin.math.round

class NimWorld(val plugin: Nimcube, val bukkitWorld: World, val dt: Float, val acceleration: Vector3f) {
    val nim = plugin.nim
    val worldIndex = nim.createWorld(dt, acceleration)
    var physicsThread: BukkitTask? = null
    var bukkitThread: BukkitTask? = null

    val potentialBodyHandles = ConcurrentLinkedDeque<Nim.PotentialBodyHandle>()
    val cuboids = mutableListOf<Cuboid>()

    fun init() {
        physicsThread = Bukkit.getScheduler().runTaskTimerAsynchronously(plugin, Runnable { physicsTick() }, 1, 1)
        bukkitThread = Bukkit.getScheduler().runTaskTimer(plugin, Runnable { bukkitTick() }, 1, 1)
    }

    fun physicsTick() {
        repeat(round(0.05 / dt).toInt()) {
            nim.tickWorld(worldIndex)
        }
    }

    fun bukkitTick() {
        var handle: Nim.PotentialBodyHandle? = null
        val toRetry = mutableListOf<Nim.PotentialBodyHandle>()
        while (potentialBodyHandles.poll().also { handle = it } != null) {
            val h = handle
            if (h == null) break
            val bodyHandle = h.tryGet()
            if (bodyHandle == null) toRetry += h
            else cuboids += Cuboid(this, bodyHandle).also { it.init() }
        }
        potentialBodyHandles += toRetry

        Arena.ofConfined().use { arena ->
            cuboids.forEach { it.update(arena) }
        }
    }

    fun deinit() {
        cuboids.forEach { it.deinit() }
        physicsThread?.cancel()
        bukkitThread?.cancel()
    }
}
