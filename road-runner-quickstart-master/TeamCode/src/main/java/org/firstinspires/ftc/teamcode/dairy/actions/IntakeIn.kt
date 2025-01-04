package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.util.Waiter

class IntakeIn() : Action {
    private var init = false
    private lateinit var waiter: Waiter

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            Intake.target = 0.0
            Intake.pidused = true
            init = true
        }

        if(Intake.intake!!.currentPosition <= 20) {
            Intake.pidused = false
        }

        return Intake.intake!!.currentPosition <= 19
    }

    companion object {
        fun intakeIn(): Action {
            return IntakeIn()
        }}
}