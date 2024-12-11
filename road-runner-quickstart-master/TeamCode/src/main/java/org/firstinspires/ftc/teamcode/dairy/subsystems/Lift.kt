package org.firstinspires.ftc.teamcode.dairy.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial.gamepad1
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.dairy.subsystems.Template.Attach
import org.firstinspires.ftc.teamcode.dairy.control.FullController
import java.lang.annotation.Inherited

@Config
class Lift private constructor() : Subsystem {
    @Retention(AnnotationRetention.RUNTIME)
    @Target(AnnotationTarget.CLASS)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    override fun postUserInitHook(opMode: Wrapper) {
        val hardwareMap = opMode.opMode.hardwareMap
        outtake1 = hardwareMap.get(DcMotorEx::class.java, "outtake1")
        outtake2 = hardwareMap.get(DcMotorEx::class.java, "outtake2")
        outtake2?.direction = DcMotorSimple.Direction.REVERSE
        
        defaultCommand = update()

        pid = FullController(
            motor = outtake1!!,
            q = k,
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
        // Additional loop logic can go here
    }

    companion object {
        val INSTANCE: Lift = Lift()
        private var outtake1: DcMotorEx? = null
        private var outtake2: DcMotorEx? = null
        private var pid: FullController? = null

        @JvmField var target:Double = 0.0

        var time = 0.0

        @JvmField var k = 0.0
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

        @JvmField var tolerance = 20

        @JvmField var pidfused:Boolean = false
    }



    fun pidUpdate() {
        pid!!.target = target.toDouble() // Set the target for FullController

        outtake1!!.let {
            val power: Double = pid!!.update() ?: 0.0
            outtake1!!.power = power
            outtake2!!.power = power
        }
    }

    fun update(): Lambda {
        return Lambda("update the pid")
            .addRequirements(Intake.INSTANCE)
            .setExecute {
                if(pidfused)
                {pidUpdate()}
            }
            .setFinish { false }
    }

    fun goTo(to: Int): Lambda {
        return Lambda("set pid target")
            .setInit {
                target = to.toDouble()
            }
            .setExecute{
                update()
            }
            .setFinish({ atTarget()})
    }

    fun atTarget(): Boolean {
        return (outtake1!!.currentPosition >= (target - tolerance) || outtake1!!.currentPosition <= (target + tolerance))
    }
}