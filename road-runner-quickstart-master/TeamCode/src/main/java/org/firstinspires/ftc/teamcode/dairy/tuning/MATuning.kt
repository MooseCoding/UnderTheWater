package org.firstinspires.ftc.teamcode.dairy.tuning

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw
import org.firstinspires.ftc.teamcode.dairy.util.MercurialAction
import org.firstinspires.ftc.teamcode.dairy.util.SilkRoad
import org.firstinspires.ftc.teamcode.old.teamcode.MecanumDrive

@OuttakeClaw.Attach
@Lift.Attach
@Mercurial.Attach
@Autonomous
@SilkRoad.Attach
class MATuning : OpMode() {
    private var d: MecanumDrive? = null
    private val init_pos = Pose2d(12.0, 61.0, -Math.PI / 2)

    private var dropOff: Action? = null
    private var dropOff2:Action? = null

    private var init:Boolean = false

    override fun init() {
        d = MecanumDrive(
            hardwareMap,
            init_pos
        )

        dropOff = d!!.actionBuilder(init_pos)
            .strafeTo(Vector2d(12.0, 40.0))
            .build()

        dropOff2 = d!!.actionBuilder(Pose2d(12.0,40.0,-Math.PI/2))
            .strafeTo(Vector2d(12.0, 37.0))
            .build()

        OuttakeClaw.claw_pos = OuttakeClaw.claw_close
    }

    override fun start() {
        SilkRoad.RunAsync(
            SequentialAction(
                MercurialAction(OuttakeClaw.INSTANCE.clawClose()),
                dropOff!!,
                MercurialAction(Lift.goTo(1300)),
                SleepAction(0.5),
                MercurialAction(OuttakeClaw.INSTANCE.pitchUp()),
                SleepAction(0.5),
                dropOff2!!,
                MercurialAction(Lift.goTo(2100)),
                SleepAction(0.3),
                MercurialAction(OuttakeClaw.INSTANCE.clawOpen()),
                //OuttakeClawClose.outtakeClawClose(),
                //OuttakePitchDown.outtakePitchDown(),
                //OuttakeClawOpen.outtakeClawOpen()
            )
        )

        init = true
    }

    override fun loop() {
        if(!init) {

        }

        d!!.updatePoseEstimate()
        telemetry.addData("d pose", d!!.pose)
        telemetry.update()
    }
}