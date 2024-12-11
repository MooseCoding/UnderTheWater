package org.firstinspires.ftc.teamcode.dairy.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial.gamepad1
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw.Companion
import org.firstinspires.ftc.teamcode.dairy.subsystems.Template.Attach
import org.firstinspires.ftc.teamcode.dairy.util.Waiter
import java.lang.annotation.Inherited

@Config
class OuttakeClaw private constructor() : Subsystem {
    @Retention(AnnotationRetention.RUNTIME)
    @Target(AnnotationTarget.CLASS)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    private val claw_open: Double = 0.0
    private val claw_close: Double = 0.0
    private val pitch_up: Double = 0.0
    private val pitch_down: Double = 0.0
    private val yaw_home: Double = 0.0
    private val claw_partial:Double =0.0


    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    override fun postUserInitHook(opMode: Wrapper) {
        val hardwareMap = opMode.opMode.hardwareMap

        claw = hardwareMap.servo["outtakeClaw"]
        pitch = hardwareMap.servo["outtakePitch"]


        time = opMode.opMode.runtime

        defaultCommand = update()
    }

    override fun postUserLoopHook(opMode: Wrapper) {
        // Additional loop logic can go here
    }

    companion object {
        private lateinit var waiter: Waiter

        val INSTANCE: OuttakeClaw = OuttakeClaw()

        private var claw: Servo? = null
        private var pitch: Servo? = null

        private var time = 0.0

        @JvmField var claw_pos: Double = 0.0
        @JvmField var pitch_pos: Double = 0.0
    }

    fun update(): Lambda {
        return Lambda("update outtake claw")
            .setRequirements(INSTANCE)
            .setExecute{
                pitch!!.position = pitch_pos
                claw!!.position = claw_pos
            }
            .setFinish{false}
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

    fun clawClose(): Lambda {
        return Lambda("claw close")
            .setRequirements(INSTANCE)
            .setInit {
                claw_pos = claw_close
                update()
                waiter.start(200)
            }
            .setFinish {
                waiter.isDone
            }
    }

    fun clawOpen(): Lambda {
        return Lambda("claw open")
            .setRequirements(INSTANCE)
            .setInit {
                claw_pos = claw_open
                update()
                waiter.start(200)
            }
            .setFinish {
                waiter.isDone
            }
    }
}