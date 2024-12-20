package org.firstinspires.ftc.teamcode.dairy.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.dairy.control.FullController
import org.firstinspires.ftc.teamcode.dairy.control.PID
import org.firstinspires.ftc.teamcode.dairy.util.Waiter
import java.lang.annotation.Inherited

@Config
class Intake private constructor() : Subsystem {
    @Retention(AnnotationRetention.RUNTIME)
    @Target(AnnotationTarget.CLASS)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> =
        Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    override fun postUserInitHook(opMode: Wrapper) {
        val hardwareMap = opMode.opMode.hardwareMap // Get the Hardware map from the current opmode
        intake = hardwareMap.get(DcMotorEx::class.java, "intake") // Get the intake motor from the HW map
        intake!!.direction = DcMotorSimple.Direction.REVERSE // Reverse the intake motor
        defaultCommand = update() // default command to run each loop

       /* pid = FullController(
            intake!!,
            q,
            r,
            n.toInt(),
            pkP,
            pkI,
            pkD,
            vkP,
            vkI,
            vkD,
            kV,
            kA,
            kS
        )*/

        pid = PID(intake!!, p, i, d) // init the pid controller

        waiter = Waiter()
    }

    override fun postUserLoopHook(opMode: Wrapper) {

    }

    companion object {
        val INSTANCE: Intake = Intake() // Static definition
        var intake: DcMotorEx? = null // Motor init


        @JvmField var pidused: Boolean = false // If true, the PID controller will run otherwise it will not

        /* Fields for using dual Kalman Filter, Dual PID, and Feedforward controller
        @JvmField var q: Double = 100.0
        @JvmField var r: Double = 1.0
        @JvmField var n: Double = 10.0

        @JvmField var pkP: Double = 0.0
        @JvmField var pkD: Double = 0.0
        @JvmField var pkI: Double = 0.0

        @JvmField var vkP: Double = 0.0
        @JvmField var vkD: Double = 0.0
        @JvmField var vkI: Double = 0.0

        @JvmField  var kV : Double= 0.0
        @JvmField  var kA : Double= 0.0
        @JvmField var kS : Double= 0.0
        */

        @JvmField var p: Double = 0.02 // proportional term of the pid controller
        @JvmField var i: Double = 0.000001 // integral term of the pid controller
        @JvmField var d: Double = 0.0003 // derivative term of the pid controller

        var pid: PID? = null // init the pid controller

        private lateinit var waiter:Waiter

        @JvmField var tolerance : Int= 10 // tolerance for the pid controller

        @JvmField var target: Double = 0.0 // target for the pid controller

        /*
        @JvmField  var targetV : Double= 0.0
        @JvmField var targetA : Double= 0.0*/

        fun pidUpdate() {
            pid!!.target = target.toInt() // Set the target for FullController

            intake!!.power = pid!!.update() // change the power of the intake motor according to the PID
        }

        fun update(): Lambda {
            return Lambda("update the pid")
                .addRequirements()
                .setExecute {
                    if(pidused)
                    {pidUpdate() }
                }
                .setFinish { false }
        }

        fun goTo(to: Int): Lambda {
            return Lambda("set pid target")
                .setInit {
                    target = to.toDouble() // set the controller target
                    pid!!.target = target.toInt() // Update target in controller
                    waiter.start(300)
                }
                .setExecute {
                    update()
                }
                .setFinish { waiter.isDone }
        }

        fun atTarget(): Boolean {
            return (intake!!.currentPosition >= (target - tolerance) && intake!!.currentPosition <= (target + tolerance)) // If intake pos is within the tolerance
        }

        fun flipPID(): Lambda {
            return Lambda("flip PID value")
                .setExecute {
                    pidused = !pidused // change the pid value to be what is it
                }
                .setFinish{true}
        }
    }
}