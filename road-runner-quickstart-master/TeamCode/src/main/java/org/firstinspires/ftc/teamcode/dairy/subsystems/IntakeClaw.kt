package org.firstinspires.ftc.teamcode.dairy.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial.gamepad1
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.dairy.subsystems.Template.Attach
import java.lang.annotation.Inherited

@Config
class IntakeClaw private constructor() : Subsystem {
    @Retention(AnnotationRetention.RUNTIME)
    @Target(AnnotationTarget.CLASS)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    override fun postUserInitHook(opMode: Wrapper) {
        val hardwareMap = opMode.opMode.hardwareMap

        claw = hardwareMap.servo["outtakeClaw"]
        pitch = hardwareMap.servo["outtakePitch"]

        defaultCommand = update()

        time = opMode.opMode.runtime
    }

    override fun postUserLoopHook(opMode: Wrapper) {
        // Additional loop logic can go here
    }

    companion object {
        val INSTANCE: IntakeClaw = IntakeClaw()

        private var claw: Servo? = null
        private var pitch: Servo? = null

        private var clawTarget = 0.0
        private var pitchTarget = 0.0

        private var time = 0.0
    }

    fun update(): Lambda {
        return Lambda("update the pid")
            .addRequirements(Lift.INSTANCE)
            .setExecute { goTo() }
            .setFinish { false }
    }

    fun goTo(): Lambda {
        return Lambda("set pid target")
            .setExecute {
                claw!!.position = clawTarget
                pitch!!.position = pitchTarget
            }
    }

    fun dropSample(): Lambda {
        return Lambda("drop sample")
            .setRequirements(INSTANCE)
            .setExecute {
                if (gamepad1.rightTrigger.state >= .1) {
                    clawTarget = 0.0
                    pitchTarget = 0.0
                }
            }
    }

    fun returnHome(): Lambda {
        return Lambda("lift down")
            .setRequirements(INSTANCE)
            .setExecute {
                if(gamepad1.leftTrigger.state >= .1) {
                    clawTarget = 0.0
                    pitchTarget = 0.0
                }
            }
    }

    fun transfer(): Lambda {
        return Lambda("outtake claw transfer")
            .setRequirements(INSTANCE)
            .setExecute {
                clawTarget = 0.0
                pitchTarget = 0.0
            }
    }
}