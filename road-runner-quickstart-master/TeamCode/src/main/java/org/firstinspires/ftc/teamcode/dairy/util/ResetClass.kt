package org.firstinspires.ftc.teamcode.dairy.util

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift

@TeleOp
@Mercurial.Attach
@Lift.Attach
@IntakeClaw.Attach
@Intake.Attach
@OuttakeClaw.Attach

class ResetClass: OpMode() {
    override fun init() {
        Sequential(
            Lift.goTo(0),
            IntakeClaw.INSTANCE.pitchUp(),
            IntakeClaw.INSTANCE.closeClaw(),
            IntakeClaw.INSTANCE.cleanYaw(),
            Intake.pidTrue(),
            Intake.goTo(0),
            Intake.pidFalse(),
            OuttakeClaw.INSTANCE.pitchDown(),
            OuttakeClaw.INSTANCE.clawClose()
        ).schedule()
    }

    override fun loop() {
        Sequential(
            Lift.goTo(0),
            IntakeClaw.INSTANCE.pitchUp(),
            IntakeClaw.INSTANCE.closeClaw(),
            IntakeClaw.INSTANCE.cleanYaw(),
            Intake.pidTrue(),
            Intake.goTo(0),
            Intake.pidFalse(),
            OuttakeClaw.INSTANCE.pitchDown(),
            OuttakeClaw.INSTANCE.clawClose()
        ).schedule()
    }
}