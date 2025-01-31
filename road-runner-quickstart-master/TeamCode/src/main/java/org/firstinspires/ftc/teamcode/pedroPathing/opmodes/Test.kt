package org.firstinspires.ftc.teamcode.pedroPathing.opmodes

import android.service.controls.Control
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
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants



@Autonomous

class Test: OpMode() {
    fun GeneratedPath(): PathChain {
        val builder: PathBuilder = PathBuilder()

        builder
            .addPath( // Line 1
                BezierCurve(
                    Point(130.526, 84.000, Point.CARTESIAN),
                    Point(115.0, 100.0, Point.CARTESIAN),
                    Point(108.211, 84.632, Point.CARTESIAN)
                )
            ).setTangentHeadingInterpolation()

        return builder.build()
    }

    private lateinit var follower: Follower
    private var pathTimer: Timer? = null
    private var opmodeTimer: Timer? = null

    private var pathState = 0

    private lateinit var path: PathChain

    override fun init() {
        pathTimer = Timer()
        opmodeTimer = Timer()

        opmodeTimer!!.resetTimer()
        Constants.setConstants(FConstants::class.java, LConstants::class.java)


        follower = Follower(hardwareMap)
        follower.setStartingPose(Pose(130.526, 84.000, Math.PI))
        follower.updatePose()
        path = GeneratedPath()
    }

    fun turn(radians:Double) {
        var temp: Pose = Pose(follower.pose.x, follower.pose.y, radians)
        follower.holdPoint(temp)
    }

    fun turnFrom(radians:Double) {
        var originalHeading: Double = follower.pose.heading + radians
        var temp: Pose = Pose(follower.pose.x, follower.pose.y, follower.pose.heading + radians)
        follower.holdPoint(temp)

        /*if(originalHeading - radians < originalHeading) {
             while(follower.pose.heading < originalHeading - Math.PI/6) {
                 follower.holdPoint(temp)
                 follower.updatePose()
             }
        }

        if(originalHeading - radians > originalHeading) {
            while(follower.pose.heading > originalHeading + Math.PI/6) {
                follower.holdPoint(temp)
                follower.updatePose()
            }
        }*/
    }


    fun setPathState(pState: Int) {
        pathState = pState
        pathTimer!!.resetTimer()
    }

    lateinit var target: Pose
    var x:Boolean = false
    var y:Boolean = false

    var i:Boolean = false

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
                if(!i) {
                    turnFrom(Math.PI)
                i = true
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

    override fun init_loop() {
        telemetry.addData("path state", pathState)
        telemetry.addData("x", follower!!.pose.x)
        telemetry.addData("y", follower!!.pose.y)
        telemetry.addData("heading", follower!!.pose.heading)
        telemetry.addData("target x", path.getPath(0).getPoint(1.0).x)
        telemetry.addData("target y", path.getPath(0).getPoint(1.0).y)

        telemetry.update()
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
        telemetry.addData("x", x)
        telemetry.addData("y",y)

        telemetry.update()
    }
}
