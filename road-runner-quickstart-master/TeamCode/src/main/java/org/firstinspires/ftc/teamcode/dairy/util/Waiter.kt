package org.firstinspires.ftc.teamcode.dairy.util

class Waiter {
    private var startTime: Long = 0
    private var waitMS: Long = 0

    fun start(waitMS: Int) {
        startTime = System.nanoTime() / 1000000
        this.waitMS = waitMS.toLong()
    }

    val isDone: Boolean
        get() = ((System.nanoTime() / 1000000) > (startTime + waitMS))
}