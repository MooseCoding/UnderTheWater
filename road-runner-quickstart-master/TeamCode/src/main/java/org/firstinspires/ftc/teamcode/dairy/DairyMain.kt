package org.firstinspires.ftc.teamcode.dairy

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.pasteurized.Pasteurized
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import dev.frozenmilk.dairy.pasteurized.layering.LayeredGamepad
import dev.frozenmilk.dairy.pasteurized.layering.MapLayeringSystem
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.dairy.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw.Companion.INSTANCE
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw.Companion.claw_pos
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw.Companion.update
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw
import org.firstinspires.ftc.teamcode.old.Drive

@Mercurial.Attach
@Drivetrain.Attach
@Lift.Attach
@OuttakeClaw.Attach
@IntakeClaw.Attach
@Intake.Attach
@TeleOp
class DairyMain: OpMode() {
    override fun init() {
        // Change controls to the other driver
        Mercurial.gamepad1.triangle.onTrue(Lambda("switch driver").setInit {
            Drivetrain.driver1 = false
        }.setFinish { true })
        Mercurial.gamepad2.triangle.onTrue(Lambda("switch driver").setInit {
            Drivetrain.driver1 = true
        }.setFinish { true })

        Mercurial.gamepad2.circle.onTrue(
            Sequential(
                IntakeClaw.INSTANCE.pitchDown(), // 200 ms
                IntakeClaw.INSTANCE.openClaw()  // 200 ms
            )
        )
        Mercurial.gamepad2.cross.onTrue(
            Sequential(
                Parallel(
                IntakeClaw.INSTANCE.closeClaw(), // 200 ms
                IntakeClaw.INSTANCE.pitchUp(), // 800
                IntakeClaw.INSTANCE.cleanYaw() // 200 ms
                ), // running parallel est 300 ms saved

                Intake.flipPID(), // 0 ms
                Intake.goTo(0), // ~300 ms
                OuttakeClaw.INSTANCE.clawClose(), // 200 ms
                IntakeClaw.INSTANCE.partialClaw() // 200 ms
            )
        )
        Mercurial.gamepad2.square.onTrue(
            Parallel(
                Lift.goTo(3900), // ~1200 ms
                OuttakeClaw.INSTANCE.pitchUp() // 200 ms
            )
        )
        Mercurial.gamepad2.rightBumper.onTrue(
            Sequential(
                OuttakeClaw.INSTANCE.clawOpen(), // 200 ms
                OuttakeClaw.INSTANCE.clawClose(), // 200 ms
                OuttakeClaw.INSTANCE.pitchDown(), // 300 ms
                Parallel(
                    IntakeClaw.INSTANCE.closeClaw(), // 200 ms
                    OuttakeClaw.INSTANCE.clawOpen(), // 200 ms
                    Lift.goTo(1000), // ~600 ms
                    Intake.flipPID() // 0 ms
                ), Lift.goTo(0) // ~300 ms
            )
        )

        Mercurial.gamepad2.dpadUp.onTrue(
            Sequential(
                IntakeClaw.INSTANCE.pitchDown(),
                IntakeClaw.INSTANCE.openClaw()
            )
        )

        Mercurial.gamepad2.dpadRight.onTrue(
            Sequential(
                IntakeClaw.INSTANCE.closeClaw(), IntakeClaw.INSTANCE.pitchUp(),
                IntakeClaw.INSTANCE.cleanYaw(), Intake.flipPID(), Intake.goTo(0), Intake.flipPID()
            )
        )

        Mercurial.gamepad2.dpadLeft.onTrue(
            Sequential(
                IntakeClaw.INSTANCE.closeClaw(), IntakeClaw.INSTANCE.pitchUp()
            )
        )

        Mercurial.gamepad2.dpadDown.onTrue(
            Sequential(
                IntakeClaw.INSTANCE.closeClaw(),
                IntakeClaw.INSTANCE.pitchUp(),
                IntakeClaw.INSTANCE.cleanYaw(),
                Intake.flipPID(),
                Intake.goTo(0),
                OuttakeClaw.INSTANCE.clawClose(),
                IntakeClaw.INSTANCE.partialClaw()
            )
        )

        Mercurial.gamepad2.leftBumper.onTrue(
            Parallel(
                Lift.goTo(1200),
                OuttakeClaw.INSTANCE.pitchUp()
            )
        )

        Mercurial.gamepad2.share.onTrue (
            Sequential(
                Lift.goTo(1300),
                OuttakeClaw.INSTANCE.clawOpen(),
                Lift.goTo(800),
                OuttakeClaw.INSTANCE.clawClose(),
                OuttakeClaw.INSTANCE.pitchDown(),
                Parallel(Lift.goTo(0),OuttakeClaw.INSTANCE.clawOpen())
            )
        )
    }

    override fun init_loop() {
        Lift.target = 0.0
    }

    override fun loop() {

        var rT:Double = Mercurial.gamepad2.rightTrigger.state
        var lT:Double = Mercurial.gamepad2.leftTrigger.state

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
            addData("intake pos", Intake.intake!!.currentPosition)
            addData("intake pid", Intake.pidused)
            addData("intake target", Intake.target)
        }
    }

}
