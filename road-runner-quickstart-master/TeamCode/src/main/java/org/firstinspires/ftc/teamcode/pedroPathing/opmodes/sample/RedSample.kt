package org.firstinspires.ftc.teamcode.pedroPathing.opmodes.sample

import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathBuilder
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants

@Autonomous

@Mercurial.Attach
@Intake.Attach
@Lift.Attach
@OuttakeClaw.Attach
@IntakeClaw.Attach

class RedSample: OpMode() {
    fun GeneratedPath(): PathChain {
        val builder: PathBuilder = PathBuilder()

        builder
            .addPath( // Line 1
                BezierLine(
                    Point(135.000, 64.000, Point.CARTESIAN),
                    Point(105.684, 64.000, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.PI)

            .addPath( // Line 2
                BezierCurve(
                    Point(105.684, 64.000, Point.CARTESIAN),
                    Point(116.421, 32.421, Point.CARTESIAN),
                    Point(129.895, 16.421, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 3
                BezierCurve(
                    Point(129.895, 16.421, Point.CARTESIAN),
                    Point(138.526, 24.000, Point.CARTESIAN),
                    Point(124.421, 23.789, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 4
                BezierCurve(
                    Point(124.421, 23.789, Point.CARTESIAN),
                    Point(127.368, 14.526, Point.CARTESIAN),
                    Point(129.263, 16.421, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 5
                BezierLine(
                    Point(129.263, 16.421, Point.CARTESIAN),
                    Point(119.579, 14.737, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 6
                BezierCurve(
                    Point(119.579, 14.737, Point.CARTESIAN),
                    Point(127.368, 19.579, Point.CARTESIAN),
                    Point(129.684, 16.842, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 7
                BezierLine(
                    Point(129.684, 16.842, Point.CARTESIAN),
                    Point(118.526, 11.579, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 8
                BezierCurve(
                    Point(118.526, 11.579, Point.CARTESIAN),
                    Point(130.526, 22.947, Point.CARTESIAN),
                    Point(129.474, 16.842, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 9
                BezierCurve(
                    Point(129.474, 16.842, Point.CARTESIAN),
                    Point(83.158, 20.632, Point.CARTESIAN),
                    Point(83.789, 34.526, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 10
                BezierCurve(
                    Point(83.789, 34.526, Point.CARTESIAN),
                    Point(123.158, 30.316, Point.CARTESIAN),
                    Point(129.474, 16.632, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 11
                BezierCurve(
                    Point(129.474, 16.632, Point.CARTESIAN),
                    Point(83.579, 17.684, Point.CARTESIAN),
                    Point(82.105, 46.947, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()

        return builder.build()
    }

    fun intake(iT:Int, yP:Double) {
        Sequential(
            Intake.pidTrue(),
            Intake.goTo(iT),
            Lambda("yaw").setExecute{IntakeClaw.yaw_pos = yP}.setFinish{true},
            IntakeClaw.INSTANCE.openClaw(),
            IntakeClaw.INSTANCE.pitchDown(),
            IntakeClaw.INSTANCE.closeClaw(),
            IntakeClaw.INSTANCE.pitchUp(),
            IntakeClaw.INSTANCE.cleanYaw(),
            Intake.goTo(0),
            OuttakeClaw.INSTANCE.clawClose(),
            IntakeClaw.INSTANCE.partialClaw(),
            Intake.pidFalse()
        ).schedule()
    }

    fun lift(lT:Int) {
        Sequential(
            Lift.pidfTrue(),
            Parallel(
                Lift.goTo(lT),
                OuttakeClaw.INSTANCE.pitchUp()
            ),
            OuttakeClaw.INSTANCE.clawOpen(),
            OuttakeClaw.INSTANCE.clawClose(),
            OuttakeClaw.INSTANCE.pitchDown(),
            Parallel(
                IntakeClaw.INSTANCE.closeClaw(), // 200 ms
                OuttakeClaw.INSTANCE.clawOpen(), // 200 ms // ~600 ms
            ),
        ).schedule()
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

        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        follower = Follower(hardwareMap)
        follower!!.setStartingPose(Pose(144-9.3, 144-83.0, Math.PI))
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
                Mercurial.runCatching {
                    OuttakeClaw.INSTANCE.pitchUp().schedule()
                }
                follower.followPath(path.getPath(0), true)
                setPathState(1)
            }

            1 -> {
                if(isClose()) {
                    Mercurial.runCatching {
                        Sequential(
                            Lift.goTo(2280),
                            OuttakeClaw.INSTANCE.clawOpen(),
                            OuttakeClaw.INSTANCE.clawClose(),
                            OuttakeClaw.INSTANCE.pitchDown(),
                            OuttakeClaw.INSTANCE.clawOpen(),
                            Intake.goTo(500),
                            IntakeClaw.INSTANCE.openClaw(),
                            IntakeClaw.INSTANCE.pitchDown(),
                            IntakeClaw.INSTANCE.closeClaw(),
                            IntakeClaw.INSTANCE.pitchUp(),
                            IntakeClaw.INSTANCE.cleanYaw(),
                            Intake.goTo(0),
                            OuttakeClaw.INSTANCE.clawClose(),
                            IntakeClaw.INSTANCE.partialClaw(),
                        ).schedule()
                    }

                    if(Intake.target.toInt() == 0 && OuttakeClaw.pitch_pos == OuttakeClaw.pitch_down) {
                        follower.followPath(path.getPath(1), true)
                        setPathState(2)
                    }
                }

            }

            2 ->   {
                if(isClose()) {
                    lift(3900)

                    if(IntakeClaw.claw_pos == IntakeClaw.claw_close) {
                        follower.followPath(path.getPath(2), true)
                        setPathState(3)
                    }
                }

            }

            3 -> {
                Sequential(
                    Lift.goTo(0),
                    Lift.pidfFalse()
                ).schedule()

                if(isClose()) {
                    intake(1500, IntakeClaw.yaw_home)

                    if(IntakeClaw.claw_pos == IntakeClaw.claw_partial) {
                        follower.followPath(path.getPath(3), true)
                        setPathState(4)
                    }
                }
            }

            4 -> {
                if(isClose()) {
                    lift(4000)

                    if(!Lift.pidfused) {
                        follower.followPath(path.getPath(4), true)
                        setPathState(5)
                    }
                }
            }

            5 -> {
                Sequential(
                    Lift.goTo(0),
                    Lift.pidfFalse()
                ).schedule()

                if(isClose()) {
                    intake(1500,IntakeClaw.yaw_home)

                    if(IntakeClaw.claw_pos == IntakeClaw.claw_partial) {
                        follower.followPath(path.getPath(5), true)
                        setPathState(6)
                    }
                }
            }

            6 -> {
                if(isClose()) {
                    lift(4000)

                    if(!Lift.pidfused) {
                        follower.followPath(path.getPath(6), true)
                        setPathState(7)
                    }
                }
            }

            7 ->  {
                Sequential(
                    Lift.goTo(0),
                    Lift.pidfFalse()
                ).schedule()

                if(isClose()) {
                    intake(1500, IntakeClaw.yaw_90)

                    if(IntakeClaw.claw_pos == IntakeClaw.claw_partial) {
                        follower.followPath(path.getPath(7), true)
                        setPathState(8)
                    }
                }
            }

            8 -> {
                if(isClose()) {
                    lift(4000)

                    if(!Lift.pidfused) {
                        follower.followPath(path.getPath(6), true)
                        setPathState(9)
                    }
                }
            }

            9 -> {
                Sequential(
                    Lift.goTo(0),
                    Lift.pidfFalse()
                ).schedule()

                if(isClose()) {
                    intake(1500, IntakeClaw.yaw_home)

                    if(IntakeClaw.claw_pos == IntakeClaw.claw_partial) {
                        follower.followPath(path.getPath(7), true)
                        setPathState(10)
                    }
                }
            }

            10 -> {
                if(isClose()) {
                    lift(4000)

                    if(!Lift.pidfused) {
                        follower.followPath(path.getPath(8), true)
                        setPathState(11)
                    }
                }
            }

            11 -> {
                Sequential(
                    Lift.goTo(0),
                    Lift.pidfFalse()
                )

                if(isClose()) {
                    Mercurial.runCatching {
                        Sequential(
                            Lift.goTo(2000),
                            OuttakeClaw.INSTANCE.pitchUp()
                        ).schedule()
                    }

                    setPathState(-1)
                }
            }
        }
    }

    override fun start() {
        Mercurial.runCatching {
            Sequential(
                OuttakeClaw.INSTANCE.clawClose(),
                Lift.goTo(1300),
                IntakeClaw.INSTANCE.closeClaw(),
            ).schedule()
        }
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