package org.firstinspires.ftc.teamcode.robot.vision;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.drivetrain.MechanumDriveClass;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

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
                        camera.setPipeline(new robotPipeline());
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
