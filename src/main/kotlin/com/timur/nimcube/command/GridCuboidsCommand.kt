package com.timur.nimcube.command

import com.timur.nimcube.NimWorld
import com.timur.nimcube.Nimcube
import org.bukkit.Bukkit
import org.bukkit.command.Command
import org.bukkit.command.CommandExecutor
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import kotlin.random.Random

class GridCuboidsCommand(private val plugin: Nimcube) : CommandExecutor {
    fun init() {
        Bukkit.getPluginCommand("nim_grid_cuboids")!!.setExecutor(this)
    }

    private val nim = plugin.nim

    override fun onCommand(
        sender: CommandSender,
        command: Command,
        label: String,
        args: Array<out String>,
    ): Boolean {
        if (sender !is Player) return true

        val world = sender.world
        val nimWorld =
            plugin.nimWorlds.getOrPut(world) {
                NimWorld(
                    plugin,
                    world,
                    plugin.Δt,
                    plugin.acceleration,
                ).also { it.init() }
            }

        val origin = sender.location
        val spacing = 3.0
        val random = Random.Default

        for (x in 0 until 10) {
            for (z in 0 until 10) {
                val velocity = Vector3f(
                    random.nextFloat() * 4f - 2f,
                    random.nextFloat() * 4f - 2f,
                    random.nextFloat() * 4f - 2f,
                )
                val potentialBodyHandle = nim.createCuboid(
                    nimWorld.worldIndex,
                    Vector3d(
                        origin.x + x * spacing,
                        origin.y,
                        origin.z + z * spacing,
                    ),
                    velocity,
                    Vector3f(),
                    Quaternionf(),
                    Vector3f(2f, 1f, 0.5f),
                    0.5f,
                )

                nimWorld.potentialBodyHandles += potentialBodyHandle
            }
        }

        return true
    }
}
