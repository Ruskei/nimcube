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

class CuboidCommand(val plugin: Nimcube) : CommandExecutor {
    fun init() {
        Bukkit.getPluginCommand("cuboid")!!.setExecutor(this)
    }

    val nim = plugin.nim

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
                    0.01f,
                    Vector3f(0f, -1f, 0f)
                ).also { it.init() }
            }
        val potentialBodyHandle = nim.createCuboid(
            nimWorld.worldIndex,
            Vector3d(
                sender.location.x,
                sender.location.y,
                sender.location.z,
            ),
            Vector3f(2f, 0f, 0f),
            Vector3f(1f, 2f, 0f),
            Quaternionf(),
            Vector3f(2f, 1f, 0.5f),
            0.5f,
        )

        nimWorld.potentialBodyHandles += potentialBodyHandle

        return true
    }
}
