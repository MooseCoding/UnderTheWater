package org.firstinspires.ftc.teamcode.dairy

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
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw
import org.firstinspires.ftc.teamcode.dairy.util.Waiter
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants

@Autonomous

@Mercurial.Attach
@Intake.Attach
@Lift.Attach
@OuttakeClaw.Attach
@IntakeClaw.Attach

class BlueSpecimen: OpMode() {
    fun GeneratedPath(): PathChain {
        val builder = PathBuilder()

        builder
            .addPath( // Line 1
                BezierLine(
                    Point(9.000, 65.000, Point.CARTESIAN),
                    Point(38.000, 65.000, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 2
                BezierLine(
                    Point(38.000, 65.000, Point.CARTESIAN),
                    Point(18.316, 47.158, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 3
                BezierLine(
                    Point(18.316, 47.158, Point.CARTESIAN),
                    Point(70.105, 23.789, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 4
                BezierLine(
                    Point(70.105, 23.789, Point.CARTESIAN),
                    Point(11.789, 23.368, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 5
                BezierLine(
                    Point(11.789, 23.368, Point.CARTESIAN),
                    Point(70.526, 23.579, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 6
                BezierLine(
                    Point(70.526, 23.579, Point.CARTESIAN),
                    Point(70.947, 11.579, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 7
                BezierLine(
                    Point(70.947, 11.579, Point.CARTESIAN),
                    Point(12.000, 12.842, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 8
                BezierLine(
                    Point(12.000, 12.842, Point.CARTESIAN),
                    Point(70.947, 11.158, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 9
                BezierLine(
                    Point(70.947, 11.158, Point.CARTESIAN),
                    Point(70.526, 6.947, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 10
                BezierLine(
                    Point(70.526, 6.947, Point.CARTESIAN),
                    Point(8.000, 6.316, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 11
                BezierCurve(
                    Point(8.000, 6.316, Point.CARTESIAN),
                    Point(41.895, 101.263, Point.CARTESIAN),
                    Point(21.684, 58.526, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 12
                BezierLine(
                    Point(21.684, 58.526, Point.CARTESIAN),
                    Point(37.474, 60.632, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 13
                BezierCurve(
                    Point(37.474, 60.632, Point.CARTESIAN),
                    Point(25.895, 68.211, Point.CARTESIAN),
                    Point(21.895, 58.526, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 14
                BezierLine(
                    Point(21.895, 58.526, Point.CARTESIAN),
                    Point(37.474, 60.842, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 15
                BezierCurve(
                    Point(37.474, 60.842, Point.CARTESIAN),
                    Point(26.947, 68.000, Point.CARTESIAN),
                    Point(21.684, 58.526, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 16
                BezierLine(
                    Point(21.684, 58.526, Point.CARTESIAN),
                    Point(37.895, 60.421, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 17
                BezierCurve(
                    Point(37.895, 60.421, Point.CARTESIAN),
                    Point(28.632, 69.263, Point.CARTESIAN),
                    Point(21.684, 59.579, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 18
                BezierLine(
                    Point(21.684, 59.579, Point.CARTESIAN),
                    Point(37.474, 61.684, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath( // Line 19
                BezierLine(
                    Point(37.474, 61.684, Point.CARTESIAN),
                    Point(21.895, 43.158, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()

        return builder.build()
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

    private lateinit var waiter:Waiter

    private var pathState = 0

    private lateinit var path:PathChain

    override fun init() {
        pathTimer = Timer()
        opmodeTimer = Timer()

        opmodeTimer!!.resetTimer()
        Constants.setConstants(FConstants::class.java, LConstants::class.java)

        follower = Follower(hardwareMap)
        follower!!.setStartingPose(Pose(9.3, 83.0, 0.0))
        path = GeneratedPath()

        waiter = Waiter()
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

            2 -> {
                if(isClose()) {
                    intake(1500, IntakeClaw.yaw_home)

                    if(Intake.atTarget() && Intake.target.toInt() == 1500) {
                        Sequential(
                            Parallel(
                                IntakeClaw.INSTANCE.pitchDown(),
                                IntakeClaw.INSTANCE.openClaw()
                            ),
                            Parallel(IntakeClaw.INSTANCE.pitchUp(),
                                IntakeClaw.INSTANCE.closeClaw())
                        ).schedule()

                        if(IntakeClaw.claw_pos == IntakeClaw.claw_close) {
                            follower.followPath(path.getPath(2), true)
                            setPathState(3)
                        }
                    }
                }
            }

            3 -> {
                if(isClose()) {
                    follower.followPath(path.getPath(3), true)
                    setPathState(4)
                }
            }

            4 -> {
                if(isClose()) {
                    follower.followPath(path.getPath(4), true)
                    setPathState(5)
                }
            }

            5 -> {
                if(isClose()) {
                    follower.followPath(path.getPath(5), true)
                    setPathState(6)
                }
            }

            6 -> {
                if(isClose()) {
                    follower.followPath(path.getPath(5), true)
                    setPathState(7)
                }
            }
            7 -> {
                if(isClose()) {
                    follower.followPath(path.getPath(5), true)
                    setPathState(8)
                }
            }

            8 -> {
                if(isClose()) {
                    follower.followPath(path.getPath(5), true)
                    setPathState(9)
                }
            }

            9 -> {
                if(isClose()) {
                    follower.followPath(path.getPath(5), true)
                    setPathState(10)
                }
            }

            10 -> {
                if(isClose()) {
                    follower.followPath(path.getPath(5), true)
                    setPathState(11)
                }
            }

            11 -> {
                if(isClose()) {
                    follower.followPath(path.getPath(5), true)
                    setPathState(12)
                }
            }

            12 -> {
                if(isClose()) {
                    if(isClose()) {
                        intake(1500, IntakeClaw.yaw_home)

                        if(Intake.atTarget() && Intake.target.toInt() == 1500) {
                            Sequential(
                                Parallel(
                                    IntakeClaw.INSTANCE.pitchDown(),
                                    IntakeClaw.INSTANCE.openClaw()
                                ),

                                Parallel(IntakeClaw.INSTANCE.pitchUp(),
                                    IntakeClaw.INSTANCE.closeClaw()),
                                IntakeClaw.INSTANCE.cleanYaw()
                            ).schedule()

                            if(IntakeClaw.claw_pos == IntakeClaw.claw_close && IntakeClaw.yaw_pos == IntakeClaw.yaw_home) {
                                follower.followPath(path.getPath(12), true)
                                setPathState(13)
                            }
                        }
                    }
                }
            }

            13 -> {
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
                        follower.followPath(path.getPath(13), true)
                        setPathState(14)
                    }
                }
            }

            14 -> {
                if(isClose()) {
                    if(isClose()) {
                        intake(1500, IntakeClaw.yaw_home)

                        if(Intake.atTarget() && Intake.target.toInt() == 1500) {
                            Sequential(
                                Parallel(
                                    IntakeClaw.INSTANCE.pitchDown(),
                                    IntakeClaw.INSTANCE.openClaw()
                                ),

                                Parallel(IntakeClaw.INSTANCE.pitchUp(),
                                    IntakeClaw.INSTANCE.closeClaw()),
                                IntakeClaw.INSTANCE.cleanYaw()
                            ).schedule()

                            if(IntakeClaw.claw_pos == IntakeClaw.claw_close && IntakeClaw.yaw_pos == IntakeClaw.yaw_home) {
                                follower.followPath(path.getPath(14), true)
                                setPathState(15)
                            }
                        }
                    }
                }
            }

            15 -> {
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
                        follower.followPath(path.getPath(15), true)
                        setPathState(16)
                    }
                }
            }

            16 -> {
                if(isClose()) {
                    if(isClose()) {
                        intake(1500, IntakeClaw.yaw_home)

                        if(Intake.atTarget() && Intake.target.toInt() == 1500) {
                            Sequential(
                                Parallel(
                                    IntakeClaw.INSTANCE.pitchDown(),
                                    IntakeClaw.INSTANCE.openClaw()
                                ),

                                Parallel(IntakeClaw.INSTANCE.pitchUp(),
                                    IntakeClaw.INSTANCE.closeClaw()),
                                IntakeClaw.INSTANCE.cleanYaw()
                            ).schedule()

                            if(IntakeClaw.claw_pos == IntakeClaw.claw_close && IntakeClaw.yaw_pos == IntakeClaw.yaw_home) {
                                follower.followPath(path.getPath(16), true)
                                setPathState(17)
                            }
                        }
                    }
                }
            }

            17 -> {
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
                        follower.followPath(path.getPath(17), true)
                        setPathState(18)
                    }
                }
            }

            19 -> {
                if(isClose()) {
                    intake(1500, IntakeClaw.yaw_home)
                }
            }
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