package org.firstinspires.ftc.teamcode.dairy.control

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.NoFeedback
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.qualcomm.robotcore.hardware.DcMotorEx
import java.util.function.DoubleSupplier
import com.ThermalEquilibrium.homeostasis.Systems.PositionVelocitySystem
import com.acmerobotics.dashboard.config.Config

@Config
class FullController(
    var motor:DcMotorEx,
    var q: Double,
    var r: Double,
    var n: Int,
    var posKP: Double,
    var posKI: Double,
    var posKD: Double,
    var velKP: Double,
    var velKI: Double,
    var velKD: Double,
    var kV: Double,
    var kA: Double,
    var kS: Double
) {
    @JvmField var target: Double = 0.0
    @JvmField var v: Double = 0.0
    @JvmField var acl: Double = 0.0

    var posCoefficients = PIDCoefficients(posKP, posKI, posKD)
    var veloCoefficients = PIDCoefficients(velKP, velKI, velKD)

    var posControl = BasicPID(posCoefficients)
    var veloControl = BasicPID(veloCoefficients)

    var motorPosition = DoubleSupplier { motor.currentPosition.toDouble() }
    var motorVelocity = DoubleSupplier { motor.velocity }

    var positionFilter = KalmanEstimator(motorPosition, q,r,n)
    var velocityFilter = KalmanEstimator(motorVelocity, q,r,n)

    var coefficientsFF = FeedforwardCoefficients(kV, kA, kS)
    var feedforward = BasicFeedforward(coefficientsFF)

    var system = PositionVelocitySystem(positionFilter, velocityFilter, feedforward, posControl, NoFeedback())

    fun update(): Double {
        return system.update(target, v, acl)
    }
}

