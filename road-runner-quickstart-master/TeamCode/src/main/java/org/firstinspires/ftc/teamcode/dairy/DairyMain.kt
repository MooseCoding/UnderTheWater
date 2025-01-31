package org.firstinspires.ftc.teamcode.dairy

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.dairy.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw.Companion.INSTANCE
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw

@Mercurial.Attach
@Drivetrain.Attach
@Lift.Attach
@OuttakeClaw.Attach
@IntakeClaw.Attach
@Intake.Attach
@TeleOp
class DairyMain: OpMode() {
    override fun init() {
        Mercurial.gamepad1.triangle.onTrue(
            Sequential(
                Lift.pidfFalse(),
                INSTANCE.pitchDown(), // 200 ms
                INSTANCE.openClaw()  // 200 ms
            )
        )
        Mercurial.gamepad1.circle.onTrue(
            Sequential(
                INSTANCE.closeClaw(), // 200 ms
                INSTANCE.pitchUp(), // 800
                INSTANCE.cleanYaw() ,// 200 ms
            )
        )

        Mercurial.gamepad1.cross.onTrue(
            Sequential(
                // running parallel est 300 ms saved

                Intake.pidTrue(), // 0 ms
                Intake.goTo(0), // ~300 ms
                OuttakeClaw.INSTANCE.clawClose(), // 200 ms
                INSTANCE.partialClaw() // 200 ms
            )
        )

        Mercurial.gamepad1.square.onTrue(
            Sequential(
                OuttakeClaw.INSTANCE.clawClose(),
                IntakeClaw.INSTANCE.partialClaw(),
            Parallel(
                Lift.pidfTrue(),
                Lift.goTo(3900), // ~1200 ms
                OuttakeClaw.INSTANCE.pitchUp() // 200 ms
            ))
        )

        Mercurial.gamepad1.dpadUp.onTrue(
                Lift.goTo(2550)
        )

        Mercurial.gamepad1.dpadDown.onTrue(
            Lift.goTo(0)
        )

        Mercurial.gamepad1.dpadLeft.onTrue(
            Lift.goTo(1900)
        )

        Mercurial.gamepad1.dpadRight.onTrue(
            Lift.goTo(1200)
        )

        Mercurial.gamepad1.rightBumper.onTrue(
            Sequential(
                OuttakeClaw.INSTANCE.clawOpen(), // 200 ms
                OuttakeClaw.INSTANCE.clawClose(), // 200 ms
                OuttakeClaw.INSTANCE.pitchDown(), // 300 ms
                Parallel(
                    INSTANCE.closeClaw(), // 200 ms
                    OuttakeClaw.INSTANCE.clawOpen(), // 200 ms // ~600 ms
                    Intake.pidFalse() // 0 ms
                ), Lift.goTo(0),
                 // ~300 ms
            )
        )
/*
        Mercurial.gamepad1.leftBumper.onTrue(
            Parallel(
                Lift.goTo(1200),
                OuttakeClaw.INSTANCE.pitchUp()
            )
        )

        Mercurial.gamepad1.share.onTrue (
            Sequential(
                Lift.goTo(1300),
                OuttakeClaw.INSTANCE.clawOpen(),
                Lift.goTo(800),
                OuttakeClaw.INSTANCE.clawClose(),
                OuttakeClaw.INSTANCE.pitchDown(),
                Parallel(Lift.goTo(0),OuttakeClaw.INSTANCE.clawOpen())
            )
        )*/
    }

    override fun init_loop() {
        Lift.target = 0.0
    }

    override fun start() {
        Lift.pidfused = true
    }

    override fun loop() {

        var rT:Double = Mercurial.gamepad1.rightTrigger.state
        var lT:Double = Mercurial.gamepad1.leftTrigger.state

        if(rT>0 && !Intake.pidused) {
            Intake.intake!!.power = 0.5
        }
        else if (lT>0 && !Intake.pidused) {
            Intake.intake!!.power = -0.5
        }
        else if(!Intake.pidused) {
            Intake.intake!!.power = 0.0
        }

        telemetry.apply {
            addData("IP pos", IntakeClaw.pitch_pos)

            addData("intake pos", Intake.intake!!.currentPosition)
            addData("intake pid", Intake.pidused)
            addData("intake target", Intake.target)
            addData("lift target", Lift.target)
            addData("om1 pos", Lift.outtake1!!.currentPosition)
            addData("om2 pos", Lift.outtake2!!.currentPosition)

            addData("pidf", Lift.pidfused)
            addData("om1p", Lift.outtake1!!.power)
            addData("om2p", Lift.outtake2!!.power)
            addData("onTarget", Lift.atTarget())

            update()
        }
    }

}
