package com.timur.nimcube.command

import com.timur.nimcube.Nimcube
import org.bukkit.Bukkit
import org.bukkit.command.Command
import org.bukkit.command.CommandExecutor
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player

class FreezeCommand(private val plugin: Nimcube) : CommandExecutor {
    fun init() {
        Bukkit.getPluginCommand("nim_freeze")!!.setExecutor(this)
    }

    override fun onCommand(
        sender: CommandSender,
        command: Command,
        label: String,
        args: Array<out String>,
    ): Boolean {
        if (sender !is Player) return true
        if (args.isNotEmpty()) {
            sender.sendMessage("Usage: /$label")
            return true
        }

        val nimWorld = plugin.nimWorlds[sender.world]
        if (nimWorld == null) {
            sender.sendMessage("No Nim world is active in this Bukkit world.")
            return true
        }

        val frozen = nimWorld.togglePhysicsFrozen()
        sender.sendMessage("Physics frozen=$frozen")
        return true
    }
}
