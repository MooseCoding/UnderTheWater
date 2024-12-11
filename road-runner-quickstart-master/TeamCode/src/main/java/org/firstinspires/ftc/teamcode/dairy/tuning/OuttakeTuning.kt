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

@Mercurial.Attach
@OuttakeClaw.Attach
@Lift.Attach
@TeleOp
class OuttakeTuning: OpMode() {
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        Mercurial.gamepad1.leftBumper.onTrue(Lambda("false pid").setExecute{Lift.pidused=false})
        Mercurial.gamepad1.rightBumper.onTrue(Lambda("true pid").setExecute{Lift.pidused=true})
        Mercurial.gamepad1.triangle.onTrue(Parallel(Lambda("reset motor").setExecute{ outtake1!!.mode = RunMode.STOP_AND_RESET_ENCODER}, Lambda("reset motor").setExecute{ outtake2!!.mode = RunMode.STOP_AND_RESET_ENCODER}))
    }

    override fun loop() {
        Lift.pid = FullController(
            motor = outtake1!!,
            q = q,
            r = r,
            n = n.toInt(),
            posKP = pkP,
            posKI = pkI,
            posKD = pkD,
            velKP = vkP,
            velKI = vkI,
            velKD = vkD,
            kV = kV,
            kA = kA,
            kS = kS
        )

        telemetry.addData("pos", outtake1!!.currentPosition)
        telemetry.addData("kalman pos", Lift.pid!!.positionFilter.measurement)
        telemetry.addData("vel", outtake1!!.velocity)
        telemetry.addData("kalman pos", Lift.pid!!.velocityFilter.measurement)

        telemetry.update()
    }
}