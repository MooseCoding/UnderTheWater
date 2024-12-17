package org.firstinspires.ftc.teamcode.dairy.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.dairy.control.FullController
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift.Companion.outtake1
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift.Companion.outtake2
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.dairy.control.PID
import org.firstinspires.ftc.teamcode.dairy.control.PIDF
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift.Companion.d
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift.Companion.f
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift.Companion.i
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift.Companion.p
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift.Companion.pidf
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw

@Mercurial.Attach
@OuttakeClaw.Attach
@Lift.Attach
@TeleOp
class OuttakeTuning: OpMode() {
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        Mercurial.gamepad1.leftBumper.onTrue(Lambda("false pid").setExecute{Lift.pidfused=false})
        Mercurial.gamepad1.rightBumper.onTrue(Lambda("true pid").setExecute{Lift.pidfused=true})
        Mercurial.gamepad1.triangle.onTrue(Parallel(Lambda("reset motor").setExecute{ outtake1!!.mode = RunMode.STOP_AND_RESET_ENCODER}, Lambda("reset motor").setExecute{ outtake2!!.mode = RunMode.STOP_AND_RESET_ENCODER}))
    }

    override fun loop() {
        Lift.pidf = PIDF(
            motor = outtake1!!,
            p,
            i,
            d,
            f
        )

        telemetry.addData("pos", outtake1!!.currentPosition)
        telemetry.addData("vel", outtake1!!.velocity)
        telemetry.addData("power", outtake1!!.power)
        telemetry.addData("pos2", outtake2!!.currentPosition)
        telemetry.addData("vel2", outtake2!!.velocity)
        telemetry.addData("power2", outtake2!!.power)

        telemetry.addData("pid", pidf!!.update())

        telemetry.update()
    }
}