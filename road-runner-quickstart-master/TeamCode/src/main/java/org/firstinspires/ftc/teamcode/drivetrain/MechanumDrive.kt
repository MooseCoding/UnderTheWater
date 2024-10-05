package org.firstinspires.ftc.teamcode.Drivetrain

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorEx


class MechanumDrive(): OpMode(){
    
    var fL : DcMotorEx = TODO()
    var fR : DcMotorEx = TODO()
    var bL : DcMotorEx = TODO()
    var bR: DcMotorEx = TODO()

    override fun init() {
         fL = hardwareMap.dcMotor["frontLeftMotor"] as DcMotorEx
         bL = hardwareMap.dcMotor["backLeftMotor"] as DcMotorEx
         fR = hardwareMap.dcMotor["frontRightMotor"] as DcMotorEx
         bR = hardwareMap.dcMotor["backRightMotor"] as DcMotorEx
    }

    override fun loop() {
        var deltaGamepadY: Float = gamepad1.left_stick_y
        var deltaGamepadRX: Float = gamepad1.right_stick_x
        var deltaGamepadX: Float = gamepad1.left_stick_x

        val d = Math.max(Math.abs(deltaGamepadY) + Math.abs(deltaGamepadX) + Math.abs(deltaGamepadRX), 1.0.toFloat()) // Denominator
        val fLP = (deltaGamepadY + deltaGamepadX+ deltaGamepadRX) / d // Front left motor power
        val bLP = (deltaGamepadY - deltaGamepadX+ deltaGamepadRX) / d // Back left motor power
        val fRP = (deltaGamepadY - deltaGamepadX- deltaGamepadRX) / d // Front right motor power
        val bRP = (deltaGamepadY + deltaGamepadX- deltaGamepadRX) / d // Back right motor power

        fL.power = fLP.toDouble()
        bL.power = bLP.toDouble()
        fR.power = fRP.toDouble()
        bR.power = bRP.toDouble()
    }

    override fun start() {
        super.start()
        this.init_loop()
    }

}