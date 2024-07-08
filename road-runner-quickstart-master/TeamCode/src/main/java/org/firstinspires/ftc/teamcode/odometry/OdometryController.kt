package org.firstinspires.ftc.teamcode.odometry
import com.qualcomm.robotcore.hardware.DcMotorEx
import kotlin.math.cos
import kotlin.math.sin

public class OdometryController{
    private val width: Double
    private val offset : Double
    private var theta: Double
    private var x: Double
    private var y: Double
    private val lOdo: DcMotorEx
    private val rOdo: DcMotorEx
    private val cOdo: DcMotorEx
    private var cRE: Int
    private var cLE: Int
    private var cCE: Int

    public constructor(w:Double, o:Double, angle:Double, cX:Double, cY:Double, lOdometer: DcMotorEx, cOdometer:DcMotorEx, rOdometer:DcMotorEx) {
        width = w
        offset = o
        theta = angle
        x = cX
        y = cY
        lOdo = lOdometer
        cOdo = cOdometer
        rOdo = rOdometer
        cRE = rOdo.currentPosition
        cLE = lOdo.currentPosition
        cCE = cOdo.currentPosition
    }

    private fun changes(dLE: Int, dRE : Int, dCE : Int) : DoubleArray {
        var phi = (dLE-dRE)/width
        var dMP = ((dLE+dRE)/2).toDouble()
        var dPP = dCE-offset*phi
        var dX = dMP*cos(theta)-dPP*sin(theta)
        var dY = dMP*sin(theta)+dPP*cos(theta)
        return doubleArrayOf(phi, dX, dY)
    }

    public fun update(): DoubleArray{
        var rE = rOdo.currentPosition
        var cE = cOdo.currentPosition
        var lE = lOdo.currentPosition

        var dRE =  rE - cRE
        var dLE = lE - cLE
        var dCE = cE - cCE
        var deltas = changes(dLE, dRE, dCE)

        x += deltas[1]
        y += deltas[2]
        theta += deltas[0]

        cLE = lE
        cCE = cE
        cRE = rE

        return doubleArrayOf(x,y,theta)
    }
}