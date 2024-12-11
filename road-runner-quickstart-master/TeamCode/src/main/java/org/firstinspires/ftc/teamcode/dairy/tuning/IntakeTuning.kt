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
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.intake
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.q
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.kA
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.kS
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.kV
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.n
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.pkD
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.pkI
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.pkP
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.r
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.vkD
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.vkI
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.vkP
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw

@Mercurial.Attach
@IntakeClaw.Attach
@Intake.Attach
@TeleOp
class IntakeTuning: OpMode() {
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        Mercurial.gamepad1.leftBumper.onTrue(Lambda("false pid").setExecute{Intake.pidused=false})
        Mercurial.gamepad1.rightBumper.onTrue(Lambda("true pid").setExecute{Intake.pidused=true})
        Mercurial.gamepad1.triangle.onTrue(Lambda("reset motor").setExecute{ intake!!.mode = RunMode.STOP_AND_RESET_ENCODER})
    }

    override fun loop() {
        Intake.pid = FullController(
            motor = intake!!,
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

        telemetry.addData("pos", intake!!.currentPosition)
        telemetry.addData("kalman pos", Intake.pid!!.positionFilter.measurement)
        telemetry.addData("vel", intake!!.velocity)
        telemetry.addData("kalman pos", Intake.pid!!.velocityFilter.measurement)

        telemetry.update()
    }
}