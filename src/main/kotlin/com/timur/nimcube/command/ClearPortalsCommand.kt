package com.timur.nimcube.command

import com.timur.nimcube.Nimcube
import org.bukkit.Bukkit
import org.bukkit.command.Command
import org.bukkit.command.CommandExecutor
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player

class ClearPortalsCommand(private val plugin: Nimcube) : CommandExecutor {
    fun init() {
        Bukkit.getPluginCommand("nim_clear_portals")!!.setExecutor(this)
    }

    private val nim = plugin.nim

    override fun onCommand(
        sender: CommandSender,
        command: Command,
        label: String,
        args: Array<out String>,
    ): Boolean {
        if (sender !is Player) return true

        val nimWorld = plugin.nimWorlds[sender.world] ?: return true
        val worldIndex = nimWorld.worldIndex

        var removed = 0
        var portalIndex = 0
        while (true) {
            val handle = nim.getPortalHandle(worldIndex, portalIndex) ?: break
            if (nim.removePortal(worldIndex, handle)) {
                removed++
            }
            portalIndex++
        }

        sender.sendMessage("Queued removal for $removed portals.")
        return true
    }
}
