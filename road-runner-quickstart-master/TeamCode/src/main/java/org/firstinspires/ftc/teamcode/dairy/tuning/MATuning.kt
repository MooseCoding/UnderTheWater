package org.firstinspires.ftc.teamcode.dairy.tuning

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
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
    private var fSPU:Action? = null

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
            .strafeTo(Vector2d(12.0, 37.7))
            .build()

        fSPU = d!!.actionBuilder(Pose2d(12.0,37.0,-Math.PI/2))
            .strafeTo(Vector2d(40.0,42.0))
            .build();

        OuttakeClaw.claw_pos = OuttakeClaw.claw_close
    }

    override fun start() {
        val fSD: Action = d!!.actionBuilder(Pose2d(40.0, 42.0, -Math.PI/2))
            .turn(Math.PI / 2 + 0.2)
            .strafeTo(Vector2d(44.0, 48.0))
            .strafeTo(Vector2d(53.5, 51.5))
            .build()


        SilkRoad.RunAsync(
            SequentialAction(
                MercurialAction(OuttakeClaw.INSTANCE.clawClose()),
                MercurialAction(Lift.goTo(1300)),
                dropOff!!,
                MercurialAction(OuttakeClaw.INSTANCE.pitchUp()),
                SleepAction(0.5),
                dropOff2!!,
                MercurialAction(Lift.goTo(2280)),
                SleepAction(1.2),
                MercurialAction(OuttakeClaw.INSTANCE.clawOpen()),
                SleepAction(0.3),
                MercurialAction(OuttakeClaw.INSTANCE.clawClose()),
                MercurialAction(OuttakeClaw.INSTANCE.pitchDown()),
                MercurialAction(Lift.goTo(0)),
                fSPU!!,
                fSD,
                MercurialAction(Lift.goTo(4000)),
                SleepAction(1.2),
                MercurialAction(OuttakeClaw.INSTANCE.pitchUp()),
                SleepAction(0.3),
                MercurialAction(OuttakeClaw.INSTANCE.clawOpen()),
                SleepAction(0.3),
                MercurialAction(OuttakeClaw.INSTANCE.pitchDown()),
                SleepAction(0.3),
                MercurialAction(OuttakeClaw.INSTANCE.clawOpen()),
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