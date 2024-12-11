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
import org.firstinspires.ftc.teamcode.dairy.util.Waiter

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

        claw = hardwareMap.servo["intakeClaw"]
        pitch = hardwareMap.servo["intakePitch"]
        yaw = hardwareMap.servo["intakeYaw"]

        defaultCommand = update()

        time = opMode.opMode.runtime
    }

    override fun postUserLoopHook(opMode: Wrapper) {
        // Additional loop logic can go here
    }

    companion object {
        private val claw_open: Double = 0.65
        private val claw_close: Double = 0.3
        private val pitch_up: Double = 0.118
        private val pitch_down: Double = 0.22
        private val yaw_home: Double = 0.39
        private val claw_partial:Double =0.4

        val INSTANCE: IntakeClaw = IntakeClaw()

        private var claw: Servo? = null
        private var pitch: Servo? = null
        private var yaw: Servo? = null

         @JvmField var claw_pos: Double = claw_close
         @JvmField var pitch_pos: Double = pitch_up
         @JvmField var yaw_pos:Double = yaw_home

        private var time = 0.0
        private lateinit var waiter: Waiter;


    }

    fun openClaw(): Lambda {
        return Lambda("open claw")
            .setRequirements(INSTANCE)
            .setInit({
                claw_pos = claw_open
                update()
                waiter.start(200)
            })
            .setFinish({
                waiter.isDone
            })
    }

    fun update(): Lambda {
        return Lambda("update the claw")
            .setRequirements(INSTANCE)
            .setExecute{
                claw!!.position = claw_pos
                yaw!!.position = yaw_pos
                pitch!!.position = pitch_pos
            }
            .setFinish{
                false
            }
    }

    fun closeClaw(): Lambda {
        return Lambda("close claw")
            .setRequirements(INSTANCE)
            .setInit({
                claw_pos = claw_close
                update()
                waiter.start(200)
            })
            .setFinish({
                waiter.isDone
            })
    }

    fun partialClaw(): Lambda {
        return Lambda("close claw")
            .setRequirements(INSTANCE)
            .setInit({
                claw_pos = claw_partial
                update()
                waiter.start(200)
            })
            .setFinish({
                waiter.isDone
            })
    }

    fun pitchUp(): Lambda {
        return Lambda("pitch up")
            .setRequirements(INSTANCE)
            .setInit({
                pitch_pos = pitch_up
                update()
                waiter.start(200)
            })
            .setFinish({
                waiter.isDone
            })
    }

    fun pitchDown(): Lambda {
        return Lambda("pitch down")
            .setRequirements(INSTANCE)
            .setInit({
                pitch_pos = pitch_down
                update()
                waiter.start(200)
            })
            .setFinish({
                waiter.isDone
            })
    }

    fun cleanYaw(): Lambda {
        return Lambda("reset yaw")
            .setRequirements(INSTANCE)
            .setInit({
                yaw_pos = yaw_home
                update()
                waiter.start(200)
            })
            .setFinish({
                waiter.isDone
            })
    }
}
