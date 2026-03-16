package com.timur.nimcube.command

import com.timur.nimcube.Nimcube
import org.bukkit.Bukkit
import org.bukkit.command.Command
import org.bukkit.command.CommandExecutor
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player

class StepCommand(private val plugin: Nimcube) : CommandExecutor {
    fun init() {
        Bukkit.getPluginCommand("nim_step")!!.setExecutor(this)
    }

    override fun onCommand(
        sender: CommandSender,
        command: Command,
        label: String,
        args: Array<out String>,
    ): Boolean {
        if (sender !is Player) return true
        if (args.size > 1) {
            sender.sendMessage("Usage: /$label [steps]")
            return true
        }

        val nimWorld = plugin.nimWorlds[sender.world]
        if (nimWorld == null) {
            sender.sendMessage("No Nim world is active in this Bukkit world.")
            return true
        }
        if (!nimWorld.physicsFrozen) {
            sender.sendMessage("Freeze physics before stepping.")
            return true
        }

        val steps = if (args.isEmpty()) 1 else args[0].toIntOrNull()
        if (steps == null || steps <= 0) {
            sender.sendMessage("Usage: /$label [steps]")
            return true
        }

        nimWorld.stepPhysicsSubsteps(steps)
        sender.sendMessage("Stepped physics by $steps substep(s).")
        return true
    }
}
