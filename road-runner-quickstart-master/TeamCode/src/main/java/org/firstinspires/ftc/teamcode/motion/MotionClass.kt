package org.firstinspires.ftc.org.teamcode.motion

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo

public class MotionClass(): OpMode() {
	enum class motionPosition{RESET, MOVING, WAIT} // Reset is at init pos, moving not at either, wait at final positions
	var aM: DcMotor = TODO() // arm motor init
	var c1: Servo = TODO() // claw servo 1
	var c2: Servo = TODO() // claw servo 2
	var rS: Servo = TODO() // rotation servo

	val initPositions: DoubleArray = doubleArrayOf(0.0,0.0,0.0,0.0) // init position of the servos/motors
	val finalPositions: DoubleArray = doubleArrayOf(1000.0,1000.0,1000.0, 1000.0) // final position

	var gamepadCopy: Gamepad = TODO() // useful for checking if any button was pressed at a point as we can make sure the action isnt repeated multiple times

	var currentMotion: motionPosition = TODO() // our current motion position

	override fun init() {
		// Initialize all our motors and servos
		aM = hardwareMap.dcMotor["armMotor"] as DcMotor
		c1 = hardwareMap.servo["claw1Servo"] as Servo
		c2 = hardwareMap.servo["claw2Servo"] as Servo
		rS = hardwareMap.servo["rotationServo"] as Servo

		// Init current motion at RESET due to it being initial
		currentMotion = motionPosition.RESET
	}
	override fun loop() {	
		when(currentMotion) {
			motionPosition.RESET -> {
				if (gamepad1.right_trigger > 0) {
					goTo(finalPositions[0], finalPositions[1], finalPositions[2], finalPositions[3])
					currentMotion = motionPosition.MOVING
				}
			}
			motionPosition.MOVING -> {
				if (aM.targetPosition > initPositions[0]  && aM.currentPosition < finalPositions[0]) {}
				else {
					if (aM.targetPosition.toDouble() == initPositions[0]) {
						currentMotion = motionPosition.RESET
					}
					else {
						currentMotion = motionPosition.WAIT
					}	
				}
			}
			motionPosition.WAIT -> {			
				if (gamepad1.left_trigger > 0) {
					goTo(initPositions[0], initPositions[1], initPositions[2], initPositions[3]) 
					currentMotion = motionPosition.MOVING
				}
			}
		}
		gamepadCopy = gamepad1
	}
	fun goTo(motorPos: Double, servo1Pos: Double, servo2Pos: Double, rotPos: Double) {
		// Set all motion elements position to their corresponding position
		aM.targetPosition = motorPos.toInt()
		aM.power = 0.8
		aM.mode = DcMotor.RunMode.RUN_TO_POSITION		

		c1.position = servo1Pos
		c2.position = servo2Pos
		rS.position = rotPos
	}
}
