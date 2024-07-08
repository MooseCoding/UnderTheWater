package org.firstinspires.ftc.teamcode.odometry

class Constants {
    private val ForditudeConsts: DoubleArray = doubleArrayOf(17.0, 5.0) //width, offset

    public fun getConsts(name: String): DoubleArray  {
        if (name.lowercase() == "ford") {
            return ForditudeConsts;
        }
        return doubleArrayOf(0.0,0.0)
    }
}