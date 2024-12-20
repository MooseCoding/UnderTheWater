package org.firstinspires.ftc.teamcode.dairy.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.dairy.control.PIDF
import java.lang.annotation.Inherited

@Config
class Lift private constructor() : Subsystem {
    @Retention(AnnotationRetention.RUNTIME)
    @Target(AnnotationTarget.CLASS)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> =
        Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    override fun postUserInitHook(opMode: Wrapper) {
        val hardwareMap = opMode.opMode.hardwareMap // Init the hardware map
        outtake1 = hardwareMap.get(DcMotorEx::class.java, "outtake1") // give outtake1 a motor
        outtake2 = hardwareMap.get(DcMotorEx::class.java, "outtake2") // give outtake 2 a motor
        outtake2?.direction = DcMotorSimple.Direction.REVERSE // flip outtake 2 to reversed

        defaultCommand = update()

        /*
        pid = FullController(
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
        */

        pidf = PIDF(outtake1!!, p, i, d, f) // give the PIDF controller a value

    }

    override fun postUserLoopHook(opMode: Wrapper) {
        // Additional loop logic can go here
    }

    companion object {
        @JvmField
        var pidfused: Boolean =
            true // if pidfused is true, the lift will run with the pid controller
        val INSTANCE: Lift = Lift() // static object
        var outtake1: DcMotorEx? = null // Init the motor to null
        var outtake2: DcMotorEx? = null // init the motor to null
        var pidf: PIDF? = null

        @JvmField
        var target: Double = 0.0

        var time = 0.0

        @JvmField
        var p: Double = 0.01
        @JvmField
        var i: Double = 0.0001
        @JvmField
        var d: Double = 0.0003
        @JvmField
        var f: Double = 0.0001

        /*

        @JvmField var q = 0.0
        @JvmField var r = 0.0
        @JvmField var n = 0.0

        @JvmField var pkP = .004
        @JvmField var pkD = .0004
        @JvmField var pkI = 0.0

        @JvmField var vkP = 0.0
        @JvmField var vkD = 0.0
        @JvmField var vkI = 0.0

        @JvmField  var kV = 0.0
        @JvmField  var kA = 0.0
        @JvmField var kS = 0.0

         */
        @JvmField
        var tolerance = 20


    fun pidUpdate() {
        pidf!!.target = target.toInt() // Set the target for FullController

        val power: Double = pidf!!.update() ?: 0.0
        outtake1!!.power = power
        outtake2!!.power = power
    }

    fun update(): Lambda {
        return Lambda("update the pid")
            .addRequirements()
            .setExecute {
                if (pidfused) {
                    pidUpdate()
                }
            }
            .setFinish { false }
    }

    fun goTo(to: Int): Lambda {
        return Lambda("set pid target")
            .setInit {
                target = to.toDouble()
                pidf!!.target = target.toInt()
            }
            .setExecute {
                update()
            }
            .setFinish { true }
    }

    fun atTarget(): Boolean {
        return (outtake1!!.currentPosition >= (target - tolerance) || outtake1!!.currentPosition <= (target + tolerance))
    }
}
}