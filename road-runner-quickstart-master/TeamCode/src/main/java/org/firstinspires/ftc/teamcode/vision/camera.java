package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrain.MechanumDriveClass;

@Photon
public class camera extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            int cID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // camera id
            OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam( hardwareMap.get(WebcamName.class, "cam"), cID); // camera object
            
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
                {
                    @Override
                    public void onOpened()
                    {
                        camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT); // CHANGE DEPENDING ON WHAT IT LOOKS LIKE
                        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                        camera.setPipeline(); 
                    }
                    @Override
                    public void onError(int errorCode)
                    {
                    /*
                    * This will be called if the camera could not be opened
                    */
                    }
            });

            waitForStart();

            while (opModeIsActive()) {

            }
        }
    }
}
