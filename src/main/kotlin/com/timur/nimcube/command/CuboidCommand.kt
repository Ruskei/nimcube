package com.timur.nimcube.command

import com.timur.nimcube.Cuboid
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
import java.lang.foreign.Arena

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

        Arena.ofConfined().use { tempArena ->
            val nimWorld =
                plugin.nimWorlds.getOrPut(world) {
                    NimWorld(
                        plugin,
                        world,
                        0.01f,
                        Vector3f(0f, -10f, 0f)
                    ).also { it.init() }
                }
            val cuboid = nim.createCuboid(
                tempArena, nimWorld.worldIndex, Vector3d(
                    sender.location.x,
                    sender.location.y,
                    sender.location.z,
                ), Vector3f(5f, 0f, 0f),
                Vector3f(0f, 3f, 0f),
                Quaternionf(),
            )
            val pos = nim.getCuboidPos(tempArena, nimWorld.worldIndex, cuboid)
            println("world: $nimWorld")
            println("cuboid: $cuboid")
            println("pos: $pos")
            val bukkitCuboid = Cuboid(nimWorld, cuboid)
            bukkitCuboid.init()
            nimWorld.cuboids.add(bukkitCuboid)
        }

        return true
    }
}
