package org.firstinspires.ftc.teamcode.dairy.control

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.NoFeedback
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.Estimator
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.NoSensor
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem

class FeedforwardController(kV: Double, kA: Double, kS: Double, kG: Double, kCos: Double) {
    val Kv: Double = kV
    val Ka: Double = kA
    val Ks: Double = kS
    val Kg: Double = kG
    val Kcos: Double = kCos
    val none: Estimator = NoSensor()
    val none2: NoFeedback = NoFeedback()
    val coefficientsEx: FeedforwardCoefficientsEx = FeedforwardCoefficientsEx(Kv,Ka,Ks,Kg,Kcos)
    val controller: FeedforwardEx = FeedforwardEx(coefficientsEx)
    var system: BasicSystem = BasicSystem(none,none2,controller)

    fun update(pos: Double, vel: Double, acl: Double){
        var command: Double = system.update(pos, vel, acl)
    }

}