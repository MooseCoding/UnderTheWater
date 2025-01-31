package org.firstinspires.ftc.teamcode.dairy.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.dairy.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.intake

import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake.Companion.pid

import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw

@Mercurial.Attach
@Drivetrain.Attach
@TeleOp
class Drive: OpMode() {
    override fun init() {
    }

    override fun loop() {

    }
}