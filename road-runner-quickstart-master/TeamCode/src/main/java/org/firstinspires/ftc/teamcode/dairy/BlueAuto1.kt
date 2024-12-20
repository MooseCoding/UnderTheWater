package org.firstinspires.ftc.teamcode.dairy

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.dairy.actions.ClawGrab
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive

@Mercurial.Attach
@Lift.Attach
@OuttakeClaw.Attach
@IntakeClaw.Attach
@Intake.Attach
@Photon
@Autonomous
class BlueAuto1: OpMode() {
    var d: MecanumDrive = TODO()
    val init_pos = Pose2d(12.0, 61.0,0.0)

    override fun init() {
        d = MecanumDrive(hardwareMap, init_pos)
    }

    override fun loop() {
        val dropOff: TrajectoryActionBuilder = d.actionBuilder(init_pos)
            .lineToY(36.0)

        val dropOffToAscent: TrajectoryActionBuilder = d.actionBuilder(d.pose)
            .lineToY(40.0)
            .strafeTo(Vector2d(35.0, 40.0))
            .strafeTo(Vector2d(35.0, 0.0))
            .turn(-Math.PI / 2)
            .lineToX(24.0)

        val firstSamplePickUp: TrajectoryActionBuilder = d.actionBuilder(d.pose)
            .strafeTo(Vector2d(48.0, 37.0))

        val firstSampleDropOff: TrajectoryActionBuilder = d.actionBuilder(d.pose)
            .turn(Math.PI / 2)
            .strafeTo(Vector2d(52.0, 60.0))

        val firstSampleToAscent: TrajectoryActionBuilder = d.actionBuilder(d.pose)
            .strafeTo(Vector2d(37.0, 0.0))
            .turn(Math.PI / -1)
            .strafeTo(Vector2d(24.0, 0.0))


        val secondSamplePickUp: TrajectoryActionBuilder = d.actionBuilder(d.pose)
            .strafeTo(Vector2d(58.0, 35.0))
            .turn(-Math.PI / 2)

        val secondSampleDropOff: TrajectoryActionBuilder = d.actionBuilder(d.pose)
            .strafeTo(Vector2d(-52.0, -60.0))
            .turn(Math.PI / 2)

        val thirdSamplePickUp: TrajectoryActionBuilder = d.actionBuilder(d.pose)
            .strafeTo(Vector2d(-55.0, -28.0))

        val thirdSampleDropOff: TrajectoryActionBuilder = d.actionBuilder(d.pose)
            .strafeTo(Vector2d(-52.0, -60.0))

        val ascent: TrajectoryActionBuilder = d.actionBuilder(d.pose)
            .strafeTo(Vector2d(-52.0, -40.0))

            .splineTo(Vector2d(-24.0, -12.0), 0.0)
            .turn(-Math.PI / 2)

        runBlocking(
             ClawGrab.clawGrab()
        )
    }

}