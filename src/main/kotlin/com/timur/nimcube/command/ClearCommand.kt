package com.timur.nimcube.command

import com.timur.nimcube.Nim
import com.timur.nimcube.Nimcube
import org.bukkit.Bukkit
import org.bukkit.command.Command
import org.bukkit.command.CommandExecutor
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player
import java.lang.foreign.Arena

class ClearCommand(private val plugin: Nimcube) : CommandExecutor {
    fun init() {
        Bukkit.getPluginCommand("nim_clear")!!.setExecutor(this)
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
        Arena.ofConfined().use { arena ->
            val count = nim.numBodies(worldIndex)
            for (i in 0 until count) {
                val body = nim.getBody(arena, worldIndex, i)
                if (!body.isSentinel() && nim.removeCuboid(worldIndex, body.handle)) {
                    removed++
                }
            }
        }

        sender.sendMessage("Queued removal for $removed bodies.")
        return true
    }
}
