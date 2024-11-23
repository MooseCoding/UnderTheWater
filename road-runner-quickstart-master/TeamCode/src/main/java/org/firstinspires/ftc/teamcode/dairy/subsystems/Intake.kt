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
import org.firstinspires.ftc.teamcode.subsystems.Template.Attach
import org.firstinspires.ftc.teamcode.dairy.control.FullController

@Config
class Intake private constructor() : Subsystem {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    override fun postUserInitHook(opMode: Wrapper) {
        val hardwareMap = opMode.opMode.hardwareMap
        intake = hardwareMap.get(DcMotorEx::class.java, "intake")
        defaultCommand = update()

        pid = FullController(
            motor = intake!!,
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

        time = opMode.opMode.runtime
    }

    override fun postUserLoopHook(opMode: Wrapper) {
        // Additional loop logic can go here
    }

    companion object {
        val INSTANCE: Intake = Intake()
        private var intake: DcMotorEx? = null

        private var pid: FullController? = null
        private const val k = 0.0
        private const val r = 0.0
        private const val n = 0.0

        private const val pkP = .004
        private const val pkD = .0004
        private const val pkI = 0.0

        private const val vkP = 0.0
        private const val vkD = 0.0
        private const val vkI = 0.0

        private const val kV = 0.0
        private const val kA = 0.0
        private const val kS = 0.0

        private var target = 0.0

        private var time = 0.0
    }

    fun pidUpdate() {
        pid?.setTarget(target.toDouble()) // Set the target for FullController

        // Check for null values before using them
        intake?.let { motor ->
            val power: Double = pid?.update(time) ?: 0.0
            intake?.power = power
        }
    }

    fun update(): Lambda {
        return Lambda("update the pid")
            .addRequirements(INSTANCE)
            .setExecute { pidUpdate() }
            .setFinish { false }
    }

    fun goTo(to: Int): Lambda {
        return Lambda("set pid target")
            .setExecute {
                target = to.toDouble()
                pid!!.setTarget(target.toDouble()) // Update target in controller
            }
    }

    fun extendOut(): Lambda {
        return Lambda("drop sample")
            .setRequirements(INSTANCE)
            .setExecute {
                if (gamepad1.rightTrigger.state >= .1) {
                    goTo(intake!!.currentPosition + 20)
                }
            }
    }

    fun returnHome(): Lambda {
        return Lambda("lift down")
            .setRequirements(INSTANCE)
            .setExecute {
                if(gamepad1.leftTrigger.state >= .1) {
                    goTo(intake!!.currentPosition - 20)
                }
            }
    }
}