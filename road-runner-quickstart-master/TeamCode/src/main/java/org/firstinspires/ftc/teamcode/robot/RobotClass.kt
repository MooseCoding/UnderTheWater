package org.firstinspires.ftc.teamcode.Drivetrain

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.drivetrain.MechanumDrive;
import org.firstinspires.ftc.org.teamcode.motion.MotionClass 


class MechanumDrive(): OpMode(){
    
    var drivetrain: MechanumDrive = TODO() 
    var motion: MotionClass = TODO() 

    override fun init() {
         drivetrain = MechanumDrive() 
         drivetrain.init() 

         motion = MotionClass()
         motion.init() 
    }

    override fun loop() {
        
    }

    override fun start() {
        super.start()
        drivetrain.init_loop()
        motion.init_loop() 
    }

}