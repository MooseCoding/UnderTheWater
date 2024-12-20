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

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    override fun postUserInitHook(opMode: Wrapper) {
        val hardwareMap = opMode.opMode.hardwareMap

        claw = hardwareMap.servo["outtakeClaw"]
        pitch = hardwareMap.servo["t"]


        time = opMode.opMode.runtime

        defaultCommand = update()

        waiter = Waiter()
    }

    override fun postUserLoopHook(opMode: Wrapper) {
        // Additional loop logic can go here
    }

    companion object {
        private val claw_open: Double = 0.9
        private val claw_close: Double = 0.53
        private val pitch_up: Double = 0.04
        private val pitch_down: Double = 0.35

        private lateinit var waiter: Waiter

        val INSTANCE: OuttakeClaw = OuttakeClaw()

        private var claw: Servo? = null
        private var pitch: Servo? = null

        private var time = 0.0

        @JvmField var claw_pos: Double = claw_open
        @JvmField var pitch_pos: Double = pitch_down
    }

    fun update(): Lambda {
        return Lambda("update outtake claw")
            .setRequirements()
            .setExecute{
                pitch!!.position = pitch_pos
                claw!!.position = claw_pos
            }
            .setFinish{false}
    }

    fun pitchUp(): Lambda {
        return Lambda("pitch up")
            .setRequirements()
            .setInit {
                pitch_pos = pitch_up
                update()
                waiter.start(200)
            }
            .setFinish {
                waiter.isDone
            }
    }

    fun pitchDown(): Lambda {
        return Lambda("pitch down")
            .setRequirements()
            .setInit {
                pitch_pos = pitch_down
                update()
                waiter.start(300)
            }
            .setFinish {
                waiter.isDone
            }
    }

    fun clawClose(): Lambda {
        return Lambda("claw close")
            .setRequirements()
            .setInit {
                claw_pos = claw_close
                update()
                waiter.start(150)
            }
            .setFinish {
                waiter.isDone
            }
    }

    fun clawOpen(): Lambda {
        return Lambda("claw open")
            .setRequirements()
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