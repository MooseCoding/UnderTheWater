package org.firstinspires.ftc.teamcode.dairy.util

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import dev.frozenmilk.mercurial.Mercurial.isScheduled
import dev.frozenmilk.mercurial.commands.Command

class MercurialAction(private val command: Command) : Action {
    private var initialized = false

    override fun run(p: TelemetryPacket): Boolean {
        val initialized = this.initialized
        if (!initialized) {
            command.schedule()
            this.initialized = true
        }
        p.addLine("Scheduled $command")
        val finished = initialized && !isScheduled(command) && command.finished()
        if (finished) this.initialized = false
        return finished
    }
}