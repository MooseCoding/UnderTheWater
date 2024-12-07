package org.firstinspires.ftc.teamcode.old;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Photon
@TeleOp
@Disabled
public class Main extends LinearOpMode {
    private DcMotorEx fL, fR, bL, bR;
    private DcMotorEx liftMotor, armMotor;

    private enum CLAW_STATE {
        STARTING,
        INTAKE,
        GRAB,
        OUTTAKE,
        HANG
    }

    private boolean hang_state = false;

    private CLAW_STATE current_claw_state;

    private int[] hang_positions = {800, 13000};
    private int[] hang_positions_final = {1000, 0};


    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            fL = (DcMotorEx) hardwareMap.dcMotor.get("fl");
            bL = (DcMotorEx) hardwareMap.dcMotor.get("bl");
            fR = (DcMotorEx) hardwareMap.dcMotor.get("fr");
            bR = (DcMotorEx) hardwareMap.dcMotor.get("br");
            armMotor = (DcMotorEx) hardwareMap.dcMotor.get("armMotor");
            liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("liftMotor");


            current_claw_state = CLAW_STATE.STARTING;

            bR.setDirection(DcMotorSimple.Direction.REVERSE);
            fR.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            waitForStart();

            while (opModeIsActive()) {
                double max;

                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                double leftFrontPower = (axial + lateral) + yaw;
                double rightFrontPower = (axial - lateral) - yaw;
                double leftBackPower = (axial - lateral) + yaw;
                double rightBackPower = (axial + lateral) - yaw;


                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                fL.setPower(leftFrontPower);
                bL.setPower(leftBackPower);
                bR.setPower(rightBackPower);
                fR.setPower(rightFrontPower);

                // FSM
                switch(current_claw_state) {
                    case HANG: {
                        if(!hang_state) {

                            liftMotor.setTargetPosition(hang_positions[1]);
                            armMotor.setTargetPosition(hang_positions[0]);

                            armMotor.setPower(0.8);
                            liftMotor.setPower(0.8);

                            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 

                            hang_state = true;
                        }

                        if(hang_state && gamepad1.dpad_down) {
                            liftMotor.setTargetPosition(hang_positions_final[1]);
                            armMotor.setTargetPosition(hang_positions_final[0]);

                            armMotor.setPower(0.8);
                            liftMotor.setPower(0.8);

                            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
                        }

                        break;
                    }
                }

                if (gamepad1.dpad_up) {
                    current_claw_state = CLAW_STATE.HANG;
                }

                telemetry.addData("am", armMotor.getCurrentPosition());
                telemetry.addData("lp", liftMotor.getCurrentPosition());


                telemetry.update();
            }
        }
    }
}
