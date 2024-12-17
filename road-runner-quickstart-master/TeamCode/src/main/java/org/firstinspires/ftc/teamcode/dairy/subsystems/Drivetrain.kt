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
import java.lang.annotation.Inherited
import kotlin.math.abs
import kotlin.math.max
import dev.frozenmilk.mercurial.Mercurial.gamepad2


@Config
class Drivetrain private constructor() : Subsystem {
    @Retention(AnnotationRetention.RUNTIME)
    @Target(AnnotationTarget.CLASS)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    override fun postUserInitHook(opMode: Wrapper) {
        val hardwareMap = opMode.opMode.hardwareMap

        fL = hardwareMap.dcMotor["frontLeft"] as DcMotorEx
        bL = hardwareMap.dcMotor["backLeft"] as DcMotorEx
        fR = hardwareMap.dcMotor["frontRight"] as DcMotorEx
        bR = hardwareMap.dcMotor["backRight"] as DcMotorEx

        fL!!.direction = DcMotorSimple.Direction.REVERSE
        bL!!.direction = DcMotorSimple.Direction.REVERSE
        bR!!.direction = DcMotorSimple.Direction.REVERSE

        defaultCommand = driveUpdate()
    }

    override fun postUserLoopHook(opMode: Wrapper) {
        // Additional loop logic can go here
    }

    companion object {
        val INSTANCE: Drivetrain = Drivetrain()

        private var fL: DcMotorEx? = null
        private var fR: DcMotorEx? = null
        private var bR: DcMotorEx? = null
        private var bL: DcMotorEx? = null

        @JvmField var driver1: Boolean = true


        fun driveUpdate(): Lambda = Lambda("Drive")
            .setExecute {
            var max: Double

            // read the gamepads
            var axial: Double = gamepad1.leftStickX.state
            var lateral: Double = gamepad1.leftStickY.state
            var yaw: Double = gamepad1.rightStickX.state

            if(!driver1) {
                axial = gamepad2.leftStickX.state
                lateral = gamepad2.leftStickY.state
                yaw = gamepad2.rightStickX.state
            }

            var leftFrontPower: Double = (axial + lateral) + yaw
            var rightFrontPower: Double = (axial - lateral) - yaw
            var leftBackPower: Double = (axial - lateral) + yaw
            var rightBackPower: Double = (axial + lateral) - yaw


            max = max(abs(leftFrontPower), abs(rightFrontPower))
            max = max(max, abs(leftBackPower))
            max = max(max, abs(rightBackPower))

            if (max > 1.0) {
                leftFrontPower /= max
                rightFrontPower /= max
                leftBackPower /= max
                rightBackPower /= max
            }

            fL!!.power = leftFrontPower
            bL!!.power = leftBackPower
            bR!!.power = rightBackPower
            fR!!.power = rightFrontPower
            }
            .setFinish {
                false
            }
    }
}