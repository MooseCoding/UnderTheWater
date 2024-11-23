package org.firstinspires.ftc.teamcode.dairy.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.dairy.pasteurized.Pasteurized
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.Mercurial.gamepad1
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.subsystems.Template.Attach

@Config
class Drivetrain private constructor() : Subsystem {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    override fun postUserInitHook(opMode: Wrapper) {
        val hardwareMap = opMode.opMode.hardwareMap

        fL = hardwareMap.dcMotor["frontLeft"] as DcMotorEx
        bL = hardwareMap.dcMotor["backLeft"] as DcMotorEx
        fR = hardwareMap.dcMotor["frontRight"] as DcMotorEx
        bR = hardwareMap.dcMotor["backRight"] as DcMotorEx

        bR!!.setDirection(DcMotorSimple.Direction.REVERSE)
        fR!!.setDirection(DcMotorSimple.Direction.REVERSE)
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
    }


    fun run(fLP: Double, fRP: Double, bLP: Double, bRP: Double): Lambda {
        return Lambda("set pid target")
            .setExecute {
                fL!!.power = fLP
                fR!!.power = fRP
                bL!!.power = bLP
                bR!!.power = bRP
            }
    }
}