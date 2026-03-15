package com.timur.nimcube.command

import com.timur.nimcube.Nimcube
import org.bukkit.Bukkit
import org.bukkit.command.Command
import org.bukkit.command.CommandExecutor
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player
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

        Arena.ofConfined().use { arena ->
            val worldIndex = nim.createWorld()
            val cuboid = nim.createCuboid(arena, worldIndex, Vector3f(10f, 20f, 30f))
            val pos = nim.getCuboidPos(arena, worldIndex, cuboid)
            println("world: $worldIndex")
            println("cuboid: $cuboid")
            println("pos: $pos")
        }

        return true
    }
}