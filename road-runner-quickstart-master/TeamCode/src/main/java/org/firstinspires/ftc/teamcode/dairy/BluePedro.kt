package org.firstinspires.ftc.teamcode.dairy

import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw
import org.firstinspires.ftc.teamcode.dairy.util.MercurialAction
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer

@Autonomous

@Mercurial.Attach
@Intake.Attach
@Lift.Attach
@OuttakeClaw.Attach
@IntakeClaw.Attach

class BluePedro: OpMode() {
    fun GeneratedPath(): PathChain {
        val builder: PathBuilder = PathBuilder()

        builder
            .addPath( // Line 1
                BezierLine(
                    Point(9.300, 83.000, Point.CARTESIAN),
                    Point(36.668, 83.000, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 2
                BezierCurve(
                    Point(36.668, 83.000, Point.CARTESIAN),
                    Point(9.587, 124.028, Point.CARTESIAN),
                    Point(26.164, 120.033, Point.CARTESIAN),
                    Point(12.583, 129.820, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 3
                BezierCurve(
                    Point(12.583, 129.820, Point.CARTESIAN),
                    Point(22.768, 118.436, Point.CARTESIAN),
                    Point(32.755, 120.433, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 4
                BezierCurve(
                    Point(32.755, 120.433, Point.CARTESIAN),
                    Point(21.171, 114.641, Point.CARTESIAN),
                    Point(12.583, 130.019, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 5
                BezierCurve(
                    Point(12.583, 130.019, Point.CARTESIAN),
                    Point(25.165, 131.218, Point.CARTESIAN),
                    Point(36.350, 131.018, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 6
                BezierCurve(
                    Point(36.350, 131.018, Point.CARTESIAN),
                    Point(26.763, 114.441, Point.CARTESIAN),
                    Point(12.383, 130.019, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 7
                BezierLine(
                    Point(12.383, 130.019, Point.CARTESIAN),
                    Point(46.136, 131.417, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 8
                BezierCurve(
                    Point(46.136, 131.417, Point.CARTESIAN),
                    Point(31.756, 104.055, Point.CARTESIAN),
                    Point(12.782, 129.820, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 9
                BezierCurve(
                    Point(12.782, 129.820, Point.CARTESIAN),
                    Point(61.115, 142.202, Point.CARTESIAN),
                    Point(60.117, 96.266, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()

        return builder.build()
    }

    private lateinit var follower: Follower
    private var pathTimer: Timer? = null
    private var opmodeTimer: Timer? = null

    private var pathState = 0

    private lateinit var path:PathChain

    override fun init() {
        pathTimer = Timer()
        opmodeTimer = Timer()

        opmodeTimer!!.resetTimer()

        follower = Follower(hardwareMap)
        follower!!.setStartingPose(Pose(9.3, 83.0, 0.0))
        path = GeneratedPath()
    }

    fun setPathState(pState: Int) {
        pathState = pState
        pathTimer!!.resetTimer()
    }

    lateinit var target: Pose
    var x:Boolean = false
    var y:Boolean = false

    fun isClose(): Boolean {
         target = Pose(path.getPath(pathState-1).getPoint(1.0).x, path.getPath(pathState-1).getPoint(1.0).y)
        var x:Boolean = false
        var y:Boolean = false

        x = if(follower.pose.x < target.x) {
            follower.pose.x > target.x - 0.2
        } else {
            follower.pose.x < target.x + 0.2
        }

        y = if(follower.pose.y < target.y) {
            follower.pose.y > target.y - 0.2
        } else {
            follower.pose.y < target.y + 0.2
        }

        return x && y
    }

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                OuttakeClaw.INSTANCE.pitchUp()
                follower.followPath(path.getPath(0))
                setPathState(1)
            }

            1 -> {
                if(isClose()) {
                    Sequential(
                        Lift.goTo(2280),
                        OuttakeClaw.INSTANCE.clawOpen(),
                        OuttakeClaw.INSTANCE.clawClose(),
                        OuttakeClaw.INSTANCE.pitchDown(),
                        OuttakeClaw.INSTANCE.clawOpen(),
                        Intake.goTo(300),
                        IntakeClaw.INSTANCE.openClaw(),
                        IntakeClaw.INSTANCE.pitchDown(),
                        IntakeClaw.INSTANCE.closeClaw(),
                        IntakeClaw.INSTANCE.pitchUp(),
                        IntakeClaw.INSTANCE.cleanYaw(),
                        Intake.goTo(0),
                        OuttakeClaw.INSTANCE.clawClose(),
                        IntakeClaw.INSTANCE.partialClaw(),
                    )

                    if(Intake.target.toInt() == 0 && OuttakeClaw.pitch_pos == OuttakeClaw.pitch_down) {
                        follower.followPath(path.getPath(1))
                        setPathState(2)
                    }
                }

            }

            2 ->   {
                if(isClose()) {
                    Sequential(
                        Parallel(
                            Lift.goTo(3900),
                            OuttakeClaw.INSTANCE.pitchUp()
                        ),
                        OuttakeClaw.INSTANCE.clawOpen(),
                        OuttakeClaw.INSTANCE.clawClose(),
                        OuttakeClaw.INSTANCE.pitchDown(),
                        Parallel(
                            IntakeClaw.INSTANCE.closeClaw(), // 200 ms
                            OuttakeClaw.INSTANCE.clawOpen(), // 200 ms // ~600 ms
                            Intake.pidFalse() // 0 ms
                        ),
                        Lift.goTo(0),
                        Lift.pidfFalse()
                    )

                    if(!Lift.pidfused) {
                        follower.followPath(path.getPath(2))
                        setPathState(3)
                    }
                }

            }

            3 -> {
                if(isClose()) {
                    Sequential(
                        Intake.goTo(300),
                        IntakeClaw.INSTANCE.openClaw(),
                        IntakeClaw.INSTANCE.pitchDown(),
                        IntakeClaw.INSTANCE.closeClaw(),
                        IntakeClaw.INSTANCE.pitchUp(),
                        IntakeClaw.INSTANCE.cleanYaw(),
                        Intake.goTo(0),
                        OuttakeClaw.INSTANCE.clawClose(),
                        IntakeClaw.INSTANCE.partialClaw(),
                        Lift.pidfTrue()
                    )

                    if(IntakeClaw.claw_pos == IntakeClaw.claw_partial) {
                        follower.followPath(path.getPath(3))
                        setPathState(4)
                    }
                }
            }

            4 -> {
                if(isClose()) {
                    Sequential(
                        Parallel(
                            Lift.goTo(3900),
                            OuttakeClaw.INSTANCE.pitchUp()
                        ),
                        OuttakeClaw.INSTANCE.clawOpen(),
                        OuttakeClaw.INSTANCE.clawClose(),
                        OuttakeClaw.INSTANCE.pitchDown(),
                        Parallel(
                            IntakeClaw.INSTANCE.closeClaw(), // 200 ms
                            OuttakeClaw.INSTANCE.clawOpen(), // 200 ms // ~600 ms
                            Intake.pidFalse() // 0 ms
                        ),
                        Lift.goTo(0),
                        Lift.pidfFalse()
                    )

                    if(!Lift.pidfused) {
                        follower.followPath(path.getPath(4))
                        setPathState(5)
                    }
                }
            }

            5 -> {
                if(isClose()) {
                    Sequential(
                        Intake.goTo(300),
                        IntakeClaw.INSTANCE.openClaw(),
                        IntakeClaw.INSTANCE.pitchDown(),
                        IntakeClaw.INSTANCE.closeClaw(),
                        IntakeClaw.INSTANCE.pitchUp(),
                        IntakeClaw.INSTANCE.cleanYaw(),
                        Intake.goTo(0),
                        OuttakeClaw.INSTANCE.clawClose(),
                        IntakeClaw.INSTANCE.partialClaw(),
                    )

                    if(IntakeClaw.claw_pos == IntakeClaw.claw_partial) {
                        follower.followPath(path.getPath(5))
                        setPathState(6)
                    }
                }
            }

            6 -> {
                if(isClose()) {
                    Sequential(
                        Parallel(
                            Lift.goTo(3900),
                            OuttakeClaw.INSTANCE.pitchUp()
                        ),
                        OuttakeClaw.INSTANCE.clawOpen(),
                        OuttakeClaw.INSTANCE.clawClose(),
                        OuttakeClaw.INSTANCE.pitchDown(),
                        Parallel(
                            IntakeClaw.INSTANCE.closeClaw(), // 200 ms
                            OuttakeClaw.INSTANCE.clawOpen(), // 200 ms // ~600 ms
                            Intake.pidFalse() // 0 ms
                        ),
                        Lift.goTo(0),
                        Lift.pidfFalse()
                    )

                    if(!Lift.pidfused) {
                        follower.followPath(path.getPath(6))
                        setPathState(7)
                    }
                }
            }

            7 ->  {
                if(isClose()) {
                    Sequential(
                        Intake.goTo(300),
                        IntakeClaw.INSTANCE.openClaw(),
                        IntakeClaw.INSTANCE.pitchDown(),
                        IntakeClaw.INSTANCE.closeClaw(),
                        IntakeClaw.INSTANCE.pitchUp(),
                        IntakeClaw.INSTANCE.cleanYaw(),
                        Intake.goTo(0),
                        OuttakeClaw.INSTANCE.clawClose(),
                        IntakeClaw.INSTANCE.partialClaw(),
                    )

                    if(IntakeClaw.claw_pos == IntakeClaw.claw_partial) {
                        follower.followPath(path.getPath(7))
                        setPathState(8)
                    }
                }
            }

            8 -> {
                if(isClose()) {
                    Sequential(
                        Parallel(
                            Lift.goTo(3900),
                            OuttakeClaw.INSTANCE.pitchUp()
                        ),
                        OuttakeClaw.INSTANCE.clawOpen(),
                        OuttakeClaw.INSTANCE.clawClose(),
                        OuttakeClaw.INSTANCE.pitchDown(),
                        Parallel(
                            IntakeClaw.INSTANCE.closeClaw(), // 200 ms
                            OuttakeClaw.INSTANCE.clawOpen(), // 200 ms // ~600 ms
                            Intake.pidFalse() // 0 ms
                        ),
                        Lift.goTo(0),
                        Lift.pidfFalse()
                    )

                    if(!Lift.pidfused) {
                        follower.followPath(path.getPath(8))
                        setPathState(9)
                    }
                }
            }

            9 -> {
                if(isClose()) {
                    Sequential(
                        Lift.goTo(2000),
                        OuttakeClaw.INSTANCE.pitchUp()
                    )

                    setPathState(-1)
                }
            }
        }
    }

    override fun start() {
        OuttakeClaw.INSTANCE.clawClose()
        Lift.goTo(1300)
    }

    override fun loop() {
        follower.update()
        autonomousPathUpdate()

        telemetry.addData("path state", pathState)
        telemetry.addData("x", follower!!.pose.x)
        telemetry.addData("y", follower!!.pose.y)
        telemetry.addData("heading", follower!!.pose.heading)
        telemetry.addData("target x", path.getPath(pathState-1).getPoint(1.0).x)
        telemetry.addData("target y", path.getPath(pathState-1).getPoint(1.0).y)
        telemetry.addData("isClose", isClose())
        telemetry.addData("x", x)
        telemetry.addData("y",y)

        telemetry.update()
    }
}