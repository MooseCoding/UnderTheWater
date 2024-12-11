package org.firstinspires.ftc.teamcode.dairy

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.pasteurized.Pasteurized
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import dev.frozenmilk.dairy.pasteurized.layering.LayeredGamepad
import dev.frozenmilk.dairy.pasteurized.layering.MapLayeringSystem
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.dairy.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw

@Mercurial.Attach
@Drivetrain.Attach
@Lift.Attach
@OuttakeClaw.Attach
@IntakeClaw.Attach
@Intake.Attach
class DairyMain: OpMode() {
    override fun init() {
        Mercurial.gamepad1.triangle.onTrue(Sequential(IntakeClaw.INSTANCE.openClaw(), IntakeClaw.INSTANCE.pitchDown()))
        Mercurial.gamepad1.circle.onTrue(Sequential(IntakeClaw.INSTANCE.closeClaw(), IntakeClaw.INSTANCE.pitchUp(), IntakeClaw.INSTANCE.cleanYaw(), Intake.flipPID(), Intake.goTo(0)))
        Mercurial.gamepad1.cross.onTrue(Sequential(OuttakeClaw.INSTANCE.clawClose(), IntakeClaw.INSTANCE.partialClaw()))
        Mercurial.gamepad1.square.onTrue(Sequential(Parallel(Lift.INSTANCE.goTo(4000), OuttakeClaw.INSTANCE.pitchUp()), Parallel(IntakeClaw.INSTANCE.closeClaw(), Intake.flipPID())))
        Mercurial.gamepad1.leftBumper.onTrue(Sequential(OuttakeClaw.INSTANCE.clawOpen(), Parallel(OuttakeClaw.INSTANCE.pitchDown(), Lift.INSTANCE.goTo(1000)), Lift.INSTANCE.goTo(0)))
    }

    override fun loop() {

    }

}
