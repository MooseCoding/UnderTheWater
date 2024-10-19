package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//import org.firstinspires.ftc.teamcode.drivetrain.MechanumDriveClass;

@Photon
public class TestingModules extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            //MechanumDrive driving = new MechanumDrive();

            //driving.init();

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            //driving.start();
            while (opModeIsActive()) {

            }
            //driving.stop();
        }
    }
}
