package org.firstinspires.ftc.teamcode.dairy

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.dairy.actions.OuttakeClawClose
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.dairy.actions.ClawReturn
import org.firstinspires.ftc.teamcode.dairy.actions.IntakeClawClose
import org.firstinspires.ftc.teamcode.dairy.actions.IntakeClawOpen
import org.firstinspires.ftc.teamcode.dairy.actions.IntakeClawPartial
import org.firstinspires.ftc.teamcode.dairy.actions.IntakeIn
import org.firstinspires.ftc.teamcode.dairy.actions.IntakeOut
import org.firstinspires.ftc.teamcode.dairy.actions.IntakePitchDown
import org.firstinspires.ftc.teamcode.dairy.actions.IntakePitchUp
import org.firstinspires.ftc.teamcode.dairy.actions.LiftHome
import org.firstinspires.ftc.teamcode.dairy.actions.OuttakeClawOpen
import org.firstinspires.ftc.teamcode.dairy.actions.OuttakePitchUp
import org.firstinspires.ftc.teamcode.dairy.actions.SampleHeight
import org.firstinspires.ftc.teamcode.dairy.actions.SpecimenHeight

@Mercurial.Attach
@Lift.Attach
@OuttakeClaw.Attach
@IntakeClaw.Attach
@Intake.Attach
@Photon
@Autonomous
class AutoTuning: OpMode() {
    var d: MecanumDrive = TODO()
    val init_pos = Pose2d(12.0, 61.0,0.0)

    override fun init() {
        d = MecanumDrive(hardwareMap, init_pos)

        runBlocking(
            OuttakeClawClose.outtakeClawClose()
        )
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

        val pickUpAndTransfer: Action = SequentialAction(
            IntakeOut.intakeOut(),
            IntakeClawOpen.intakeClawOpen(),
            IntakePitchDown.intakePitchDown(),
            IntakeClawClose.intakeClawClose(),
            IntakePitchUp.intakePitchUp(),
            IntakeIn.intakeIn(),
            OuttakeClawClose.outtakeClawClose(),
            IntakeClawPartial.intakeClawPartial(),
        )

        val scoreSample: Action = SequentialAction(
            ParallelAction(
                SampleHeight.sampleHeight(),
                OuttakePitchUp.outtakePitchUp()
            ),
            OuttakeClawOpen.outtakeClawOpen(),
            ClawReturn.clawReturn(),
            LiftHome.liftHome()
        )


        runBlocking(
            SequentialAction(
                dropOff.build(),
                OuttakePitchUp.outtakePitchUp(),
                SpecimenHeight.specimenHeight(),
                OuttakeClawOpen.outtakeClawOpen(),
                ClawReturn.clawReturn(),
                LiftHome.liftHome() // Done with the specimen
            )
        )
    }

}