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
import java.lang.annotation.Inherited

@Config
class Intake private constructor() : Subsystem {
    @Retention(AnnotationRetention.RUNTIME)
    @Target(AnnotationTarget.CLASS)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)


    override fun postUserInitHook(opMode: Wrapper) {
        val hardwareMap = opMode.opMode.hardwareMap
        intake = hardwareMap.get(DcMotorEx::class.java, "intake")
        intake!!.direction = DcMotorSimple.Direction.REVERSE
        defaultCommand = update()

        pid = FullController(
            motor = intake!!,
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
    }

    override fun postUserLoopHook(opMode: Wrapper) {
    }

    companion object {
        val INSTANCE: Intake = Intake()
        var intake: DcMotorEx? = null

        @JvmField var pidused: Boolean = true

        @JvmField var q: Double = 0.0
        @JvmField var r: Double = 0.0
        @JvmField var n: Double = 0.0

       @JvmField var pkP: Double = 0.0
        @JvmField var pkD: Double = 0.0
         @JvmField var pkI: Double = 0.0

        @JvmField var vkP: Double = 0.0
        @JvmField var vkD: Double = 0.0
        @JvmField var vkI: Double = 0.0

        @JvmField  var kV : Double= 0.0
        @JvmField  var kA : Double= 0.0
        @JvmField var kS : Double= 0.0

        @JvmField var tolerance : Int= 20

        var target: Double = 0.0

        var time: Double = 0.0

        var pid: FullController? = null

        fun pidUpdate() {
            pid!!.target = target.toDouble() // Set the target for FullController

            intake?.let { motor ->
                val power: Double = pid!!.update() ?: 0.0
                intake?.power = power
            }
        }

        fun update(): Lambda {
            return Lambda("update the pid")
                .addRequirements(INSTANCE)
                .setExecute {
                    if(pidused)
                    {pidUpdate() }
                }
                .setFinish { false }
        }

        fun goTo(to: Int): Lambda {
            return Lambda("set pid target")
                .setExecute {
                    target = to.toDouble()
                    pid!!.target = target.toDouble() // Update target in controller
                }
                .setFinish({ atTarget()})
        }

        fun atTarget(): Boolean {
            return (intake!!.currentPosition >= (target - tolerance) || intake!!.currentPosition <= (target + tolerance))
        }

        fun flipPID(): Lambda {
            return Lambda("flip PID value")
                .setExecute {
                    pidused = !pidused
                }
                .setFinish{true}
        }

    }


}