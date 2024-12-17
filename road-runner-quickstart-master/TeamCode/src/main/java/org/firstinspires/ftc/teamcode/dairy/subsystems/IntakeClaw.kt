package org.firstinspires.ftc.teamcode.dairy.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.lang.annotation.Inherited
import org.firstinspires.ftc.teamcode.dairy.util.Waiter

@Config
class IntakeClaw private constructor() : Subsystem {
    @Retention(AnnotationRetention.RUNTIME)
    @Target(AnnotationTarget.CLASS)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    var claw: Servo? = null // Claw init
    var pitch: Servo? = null // Pitch init
    var yaw: Servo? = null // Yaw init

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    override fun postUserInitHook(opMode: Wrapper) {
        val hardwareMap = opMode.opMode.hardwareMap // Get the HW map from the current opmode

        telemetry = opMode.opMode.telemetry

        claw = hardwareMap.servo["intakeClaw"] // setup the claw
        pitch = hardwareMap.servo["intakePitch"] // setup the pitch
        yaw = hardwareMap.servo["intakeYaw"] // setup the yaw

        defaultCommand = update() // set the default command to update the servos pos

        waiter = Waiter()
    }

    override fun postUserLoopHook(opMode: Wrapper) {
        // Additional loop logic can go here
    }

    fun openClaw(): Lambda {
        return Lambda("open claw")
            .setRequirements()
            .setInit {
                claw_pos = claw_open // Open the claw pos
                update() // Force an update
                waiter.start(200)
            }
            .setFinish{
                waiter.isDone

            }
    }

    fun closeClaw(): Lambda {
        return Lambda("close claw")
            .setRequirements()
            .setInit {
                claw_pos = claw_close // set the claw pos to close
                update() // Force update
                waiter.start(200)
            }
            .setFinish {
                waiter.isDone
            }
    }

    fun partialClaw(): Lambda {
        return Lambda("close claw")
            .setRequirements()
            .setInit {
                claw_pos = claw_partial // Partially open the claw
                update() // force update
                waiter.start(200)
            }
            .setFinish {
                waiter.isDone
            }
    }

    fun pitchUp(): Lambda {
        return Lambda("pitch up")
            .setRequirements()
            .setInit {
                pitch_pos = pitch_up // set the pitch up, at home
                update()
                waiter.start(800)
            }
            .setFinish {
                waiter.isDone
            }
    }

    fun pitchDown(): Lambda {
        return Lambda("pitch down")
            .setRequirements()
            .setInit {
                pitch_pos = pitch_down // set the pitch pos to go down
                update()
                waiter.start(200)
            }
            .setFinish {
                waiter.isDone
            }
    }

    fun cleanYaw(): Lambda {
        return Lambda("reset yaw")
            .setRequirements()
            .setInit{
                yaw_pos = yaw_home // reset the yaw
                update()
                waiter.start(200)
            }
            .setFinish {
                waiter.isDone
            }
    }

    companion object {
        private val claw_open: Double = 0.9 // TUNED - Dec 12
        private val claw_close: Double = 0.6 // TUNED - Dec 12
        private val pitch_up: Double = 0.118 // TUNED - Dec 12
        private val pitch_down: Double = 0.22 // TUNED - Dec 12
        private val yaw_home: Double = 0.39 // TUNED - Dec 12
        private val claw_partial: Double = 0.74 // TUNED - Dec 12

        var telemetry:Telemetry? = null

        val INSTANCE: IntakeClaw = IntakeClaw() // Static instiantion

        @JvmField
        var claw_pos: Double = claw_close // Claw pos starts closed

        @JvmField
        var pitch_pos: Double = pitch_up // Pitch pos starts up, which is at home

        @JvmField
        var yaw_pos: Double = yaw_home // Yaw pos stays neutral

        private lateinit var waiter: Waiter

        fun update(): Lambda {
            return Lambda("update the claw")
                .setRequirements(INSTANCE)
                .setExecute {
                    INSTANCE.claw!!.position = claw_pos // Update the claw position
                    INSTANCE.yaw!!.position = yaw_pos // Update the yaw position
                    INSTANCE.pitch!!.position = pitch_pos // Update the pitch position
                }
                .setFinish {
                    false // never really finish
                }
        }
    }
}
