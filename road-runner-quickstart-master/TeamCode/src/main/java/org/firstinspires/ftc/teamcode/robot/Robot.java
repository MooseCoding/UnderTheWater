package org.firstinspires.ftc.teamcode.robot; 

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.Sample;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Robot {
    // Lynx modules
    final int TEST_CYCLES = 500;
    private int cycles = 0; 
    private double t1; 
    private ElapsedTime timer = new ElapsedTime();
    private double e1,e2,e3,e4;
    private double v1,v2,v3,v4;

    // Robot motors & servos
    private DcMotorEx fL, fR, bL, bR;
    private DcMotorEx iM, oM1, oM2;
    private Servo intakeClaw, intakeYaw, intakePitch;
    private Servo outtakeClaw, outtakePitch; 

    // Vision detection vars
    private boolean aligned = false; 

    // Temp Timer variable
    private double t = -1; 

    // Endgame Unlock 
    private boolean ENDGAME = false;

    private enum CLAW_STATE {
        INTAKE,
        TRANSFER,
        OUTTAKE,
        HANG
    }

    private enum INSTATE_INTAKE {
        OUT, 
        GRAB,
        MOVEBACK,
    }

    private enum INSTATE_TRANSFER {
        TRANSFER1,
        TRANSFER2, 
    }

    private enum INSTATE_OUTTAKE {
        LIFT, 
        TURN,
        DROP,
    }
    
    private CLAW_STATE current_claw_state = CLAW_STATE.INTAKE; 
    private MecanumDrive d; 

    private INSTATE_INTAKE ii = INSTATE_INTAKE.OUT;
    private INSTATE_TRANSFER it = INSTATE_TRANSFER.TRANSFER1; 
    private INSTATE_OUTTAKE io = INSTATE_OUTTAKE.LIFT;  

    private List<LynxModule> hubs = new List<>(); 

    public void init(HardwareMap hardwareMap) {
        d = new MecanumDrive(hardwareMap); 

        hubs = hardwareMap.getAll(LynxModule.class);

        for(LynxModule hub: hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);  
        }


        fL = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        bL = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
        fR = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        bR = (DcMotorEx) hardwareMap.dcMotor.get("backRight");
        iM = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        oM1 = (DcMotorEx) hardwareMap.dcMotor.get("outtake1");
        oM2 = (DcMotorEx) hardwareMap.dcMotor.get("outtake2");

        intakeClaw = hardwareMap.servo.get("clawServo");
        intakePitch = hardwareMap.servo.get("pitchServo");
        intakeYaw = hardwareMap.servo.get("yawServo");

        outtakeClaw = hardwareMap.servo.get("outtakeClaw");
        outtakePitch = hardwareMap.servo.get("outtakePitch");

        bR.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        iM.setDirection(DcMotorSimple.Direction.REVERSE);
        oM2.setDirection(DcMotorSimple.Direction.REVERSE);
        
        iM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        oM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        oM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        oM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        oM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void DriveLift(double power, int position) {
        oM1.setTargetPosition(position);
        oM2.setTargetPosition(position);

        oM1.setPower(power);
        oM2.setPower(power);

        oM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        oM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean alignClaw(ArrayList<Sample> samples) {
        return true;
    }

    public void ExtendIntake(double power, int position) {
        iM.setTargetPosition(position);
        iM.setPower(power);
        iM.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
    }

    public void Transition() {
         
    }

    public void loop_func(Gamepad g, ArrayList<Sample> samples, double RUNTIME) {
        // Clearing the hub bulk cache
        for(LynxModule module: hubs) {
            module.clearBulkCache(); 
        }

        // Drivetrain Movement Code
        double max;

        double axial = -g.left_stick_y;
        double lateral = g.left_stick_x;
        double yaw = g.right_stick_x;

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

        // Intake & Outake Handlers
        switch(current_claw_state) {
            case INTAKE: {
                switch(ii) {
                    case OUT: {
                        if(g.cross) {
                            intakePitch.setPosition(0.0);
                            ii = INSTATE_INTAKE.GRAB; 
                        }

                        if(g.right_trigger > 0) {
                            iM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            iM.setPower(0.5);
                        }
                        else if(g.left_trigger > 0) {
                            iM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            iM.setPower(-0.5);
                        }
                        else {
                            iM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            iM.setPower(0);
                        }

                        break;
                    }
                    case GRAB: {
                        aligned = alignClaw(samples); 

                        if(aligned && g.right_bumper) {
                            intakeClaw.setPosition(0.0);
                            t = RUNTIME; 
                            ii = INSTATE_INTAKE.MOVEBACK; 
                        }

                        break; 
                    }
                    case MOVEBACK: {
                        if(!(iM.getCurrentPosition()<50) && (RUNTIME - t > 0.3)) {
                            intakePitch.setPosition(0.0);
                            intakeYaw.setPosition(0.0);

                            iM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            iM.setTargetPosition(0);
                            iM.setPower(0.6);
                            iM.setMode(DcMotor.RunMode.RUN_TO_POSITION); 

                            t = -1;
                        }
                        else {
                            ii = INSTATE_INTAKE.OUT;
                            current_claw_state = CLAW_STATE.TRANSFER; 
                        }

                        break; 
                    }
                }
            }
            case TRANSFER: {
                switch(it) {
                    case TRANSFER1: {
                        outtakePitch.setPosition(0.0); // Resting at home
                        
                        if(t==-1) {
                            t = RUNTIME;
                        }

                        if(t != -1 && RUNTIME - t > 0.4) {
                            outtakeClaw.setPosition(0.0); // Close the claw on the sample
                            t = -1;
                            it = INSTATE_TRANSFER.TRANSFER2; 
                        }

                        break;
                    }
                    case TRANSFER2: {   
                        intakeClaw.setPosition(0.0); // Release the sample

                        if (t == -1) {
                            outtakePitch.setPosition(0.0); // Move the outtake pitch back up
                            t = RUNTIME;
                        }   

                        if (t != -1 && RUNTIME - t > 0.3 && g.cross) {
                            it = INSTATE_TRANSFER.TRANSFER1;
                            current_claw_state = CLAW_STATE.OUTTAKE; 
                        }
                        break; 
                    }   
                }

                break; 
            }
            case OUTTAKE: {
                switch(io){
                    case LIFT: {
                        DriveLift(1, 3000);

                        if(oM1.getCurrentPosition() >= 2900) {
                            io = INSTATE_OUTTAKE.TURN;
                        }

                        break; 
                    }
                    case TURN: {
                        outtakePitch.setPosition(0.0); // Making sure that the pitch is aligned to drop it

                        if (t==-1) {
                            t = RUNTIME;
                        }

                        if (t!=-1 && RUNTIME - t > 0.3 && g.cross) {
                            t = -1;
                            io = INSTATE_OUTTAKE.DROP; 
                        }

                        break;
                    }
                    case DROP: {
                        outtakeClaw.setPosition(0.0); // Drop the sample into the thingy
                        
                        if(t==-1) {
                            t = RUNTIME;
                        }

                        if(t != -1 && (RUNTIME - t > 0.3)) {
                            t = -1;
                            outtakeClaw.setPosition(0.0); // Reset the position
                            outtakePitch.setPosition(0.0); // Reset the position
                            DriveLift(0.8, 0); // Set the lift back to 0

                            io = INSTATE_OUTTAKE.LIFT;
                            current_claw_state = CLAW_STATE.INTAKE;
                        }

                        break;
                    }
                }
            }
            case HANG: {
                break;
            }
        }

        if(g.dpad_up || RUNTIME > 90) {
            ENDGAME = true; 
        }
    }

    public void axon_loop(Gamepad g) {

    }

    public void displayCycleTimes(String status) {
        telemetry.addData("Testing", status);
        telemetry.addData("CACHE = MANUAL", "%5.1f mS/cycle", t1);
        telemetry.update(); 
    }

    public void testLynx(HardwareMap hardwareMap) {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        fL = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        bL = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
        fR = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        bR = (DcMotorEx) hardwareMap.dcMotor.get("backRight");

        displayCycleTimes("Manual Lynx Control");

        for(LynxModule module: hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        timer.reset();
        cycles = 0;

        while(cycles++ < TEST_CYCLES) {
            for(LynxModule module: hubs) {
                module.clearBulkCache(); 
            }

            e1 = fL.getCurrentPosition();
            e2 = fR.getCurrentPosition();
            e3 = bL.getCurrentPosition();
            e4 = bR.getCurrentPosition(); 

            v1 = fL.getVelocity();
            v2 = fR.getVelocity();
            v3 = bL.getVelocity();
            v4 = bR.getVelocity();
        }

        t1 = timer.milliseconds() / cycles;

        displayCycleTimes("Complete"); 
    }
}