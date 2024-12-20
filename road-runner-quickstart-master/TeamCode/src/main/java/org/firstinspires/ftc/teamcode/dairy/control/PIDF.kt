package org.firstinspires.ftc.teamcode.dairy.control

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients

@Config
class PIDF(
    var motor: DcMotorEx,
    p:Double,
    i:Double,
    d:Double,
    f:Double
) {
    // creation of the PID object
    var controller = PIDFController(p, i, d, f)

    var target:Int = 0

    fun update(): Double {
        return controller.calculate( motor.currentPosition.toDouble(), target.toDouble());
    }
}