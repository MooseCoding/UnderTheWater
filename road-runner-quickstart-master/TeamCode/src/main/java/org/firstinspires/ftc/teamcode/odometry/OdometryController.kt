package org.firstinspires.ftc.teamcode.odometry
import com.qualcomm.robotcore.hardware.DcMotorEx
import kotlin.math.cos
import kotlin.math.sin

public class OdometryController{
    private val width: Double //width between two parallel odo wheels
    private val offset : Double //offset between perpendicular wheel and center of mass
    private var theta: Double //current angle of robot
    private var x: Double //current x-pos relative to origin x passed in as a parameter
    private var y: Double //current y-pos relative to the origin y passed in as a parameter
    private val lOdo: DcMotorEx //left odo DcMotorEx, only use the .currentPosition until its a controller
    private val rOdo: DcMotorEx //right odo DcMotorEx, only use the .currentPosition until its a controller
    private val cOdo: DcMotorEx //center odo DcMotorEx, only use the .currentPosition until its a controller
    private var cRE: Int //current right encoder value, used for retrieving info without calling odoWheel.currentPosition
    private var cLE: Int //current left encoder value
    private var cCE: Int //current center encoder value

    public constructor(w:Double, o:Double, angle:Double, cX:Double, cY:Double, lOdometer: DcMotorEx, cOdometer:DcMotorEx, rOdometer:DcMotorEx) {
        width = w
        offset = o
        theta = angle
        x = cX //origin x pos
        y = cY //origin y pos
        lOdo = lOdometer
        cOdo = cOdometer
        rOdo = rOdometer
        cRE = rOdo.currentPosition
        cLE = lOdo.currentPosition
        cCE = cOdo.currentPosition
    }

    private fun changes(dLE: Int, dRE : Int, dCE : Int) : DoubleArray { //math handler function
        var phi = (dLE-dRE)/width //delta theta, eg change in the angle of the robot
        var dMP = ((dLE+dRE)/2).toDouble() //delta middle position, eg change in the encoders (left and right) averaged out
        var dPP = dCE-offset*phi //delta perpendicular position, eg change in the perpendicular position based on delta theta
        var dX = dMP*cos(theta)-dPP*sin(theta) //delta x, eg change in x value over a period of time
        var dY = dMP*sin(theta)+dPP*cos(theta) //delta y, eg change in y value over a period of time
        return doubleArrayOf(phi, dX, dY) //return the deltas (our changes)
    }

    public fun update(): DoubleArray{
        var rE = rOdo.currentPosition //fetch current right odo position and store in local memory
        var cE = cOdo.currentPosition //fetch current center odo position and store in local memory
        var lE = lOdo.currentPosition //fetch current left odo position and store in local memory

        var dRE =  rE - cRE //delta right encoder, eg change in right encoder value
        var dLE = lE - cLE //delta left encoder, eg change in left encoder value
        var dCE = cE - cCE //delta center encoder, eg change in center encoder value
        var deltas = changes(dLE, dRE, dCE) //get changes with call to changes func

        x += deltas[1] //deltas[1] = dX which is the change in x which added to x gets current x pos
        y += deltas[2] //deltas[2] = dY which is the change in y which added to y gets current y pos
        theta += deltas[0] //deltas[0] = dO which is the change in theta which added to theta gets current angle

        cLE = lE //set the current left encoder to the left encoder value
        cCE = cE //set the current center encoder value to the center encoder value
        cRE = rE //set the current right encoder value to the right encoder value

        return doubleArrayOf(x,y,theta) //return the current pose of the robot
    }
}