package org.firstinspires.ftc.teamcode.dairy.control

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.qualcomm.robotcore.hardware.DcMotorEx
import java.util.function.DoubleSupplier
import com.ThermalEquilibrium.homeostasis.Systems.PositionVelocitySystem

class FullController(
    motor: DcMotorEx,
    q: Double,
    r: Double,
    n: Int,
    posKP: Double,
    posKI: Double,
    posKD: Double,
    velKP: Double,
    velKI: Double,
    velKD: Double,
    kV: Double,
    kA: Double,
    kS: Double
) {
    private var system: PositionVelocitySystem
    var motor: DcMotorEx
    var target: Double = 0.0
    var v1: Double? = null
    var v2: Double? = null
    var t1: Double? = null
    var t2: Double? = null
    var acl: Double? = null
    var i: Int = 0
    var motorVelocity: DoubleSupplier
    var motorPosition: DoubleSupplier

    init {
        this.motor = motor

        var Q: Double = q
        var R: Double = r
        var N: Int = n

        var posCoefficients = PIDCoefficients(posKP, posKI, posKD)
        var veloCoefficients = PIDCoefficients(velKP, velKI, velKD)

        var posControl = BasicPID(posCoefficients)
        var veloControl = BasicPID(veloCoefficients)

        motorPosition = DoubleSupplier { motor.currentPosition.toDouble() ?: 0.0 }
        motorVelocity = DoubleSupplier { motor.velocity ?: 0.0 }

        var positionFilter = KalmanEstimator(motorPosition, Q, R, N)
        var velocityFilter = KalmanEstimator(motorVelocity, Q, R, N)

        var coefficientsFF = FeedforwardCoefficients(kV, kA, kS)
        var feedforward = BasicFeedforward(coefficientsFF)

        system = PositionVelocitySystem(positionFilter, velocityFilter, feedforward, posControl, veloControl)
    }

    fun update(currentTime: Double): Double {
        if (acl != null) {
            calcAcl(currentTime);
            return system.update(motorPosition.asDouble, motorVelocity.asDouble, acl!!)
        }
        else {
            calcAcl(currentTime);
            return system.update(motorPosition.asDouble, motorVelocity.asDouble, 0.0)
        }
    }

    fun calcAcl(currentTime: Double) {
        if(i == 0) {
            v1 = motor.velocity
            i++
            t1 = currentTime
        }
        else if (i == 1) {
            v2 = motor.velocity
            i--
            t2 = currentTime
        }

        if(v1 != null && v2 != null) {
            acl = (v2!!-v1!!)/(t2!!-t1!!)
        }
    }
}

