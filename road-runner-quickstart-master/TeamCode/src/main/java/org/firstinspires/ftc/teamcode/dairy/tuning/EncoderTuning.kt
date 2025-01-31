package org.firstinspires.ftc.teamcode.dairy.tuning

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.dairy.subsystems.Drivetrain

@Mercurial.Attach
@TeleOp
@Drivetrain.Attach
@Disabled

class EncoderTuning:OpMode() {
    override fun init() {

    }

    override fun loop() {
        telemetry.apply {
            addData("frontLeft", Drivetrain.fL!!.currentPosition)
            addData("frontRight", Drivetrain.fR!!.currentPosition)
            addData("backRight", Drivetrain.bR!!.currentPosition)
            update()
        }
    }
}