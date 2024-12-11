package org.firstinspires.ftc.teamcode.dairy.control

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
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
    var motor: DcMotorEx,
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
    var system: PositionVelocitySystem = TODO()

    var target: Double = 0.0
    var v: Double = 0.0
    var acl: Double = 0.0

    var positionFilter:KalmanEstimator = TODO()
    var velocityFilter:KalmanEstimator = TODO()


    init {
        var posCoefficients = PIDCoefficients(posKP, posKI, posKD)
        var veloCoefficients = PIDCoefficients(velKP, velKI, velKD)

        var posControl = BasicPID(posCoefficients)
        var veloControl = BasicPID(veloCoefficients)

        var motorPosition = DoubleSupplier { motor.currentPosition.toDouble() ?: 0.0 }
        var motorVelocity = DoubleSupplier { motor.velocity ?: 0.0 }

        positionFilter = KalmanEstimator(motorPosition, q,r,n)
        velocityFilter = KalmanEstimator(motorVelocity, q,r,n)

        var coefficientsFF = FeedforwardCoefficients(kV, kA, kS)
        var feedforward = BasicFeedforward(coefficientsFF)

        system = PositionVelocitySystem(positionFilter, velocityFilter, feedforward, posControl, veloControl)
    }

    fun update(): Double {
        return system.update(target, v, acl)
    }
}

