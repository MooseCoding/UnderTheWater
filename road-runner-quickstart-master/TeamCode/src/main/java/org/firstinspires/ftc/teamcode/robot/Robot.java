package org.firstinspires.ftc.teamcode.robot; 

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.robot.Intake.pitchDown;
import static org.firstinspires.ftc.teamcode.robot.Intake.pitchUp;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.vision.Sample;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


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

    // Subsystems
    private Lift l;
    private Intake i;
    private Hang h;

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

    private List<LynxModule> hubs;

    public void init(HardwareMap hardwareMap) {
        l = new Lift(hardwareMap);
        i = new Intake(hardwareMap);
        h = new Hang(hardwareMap);

        d = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        hubs = hardwareMap.getAll(LynxModule.class);

        for(LynxModule hub: hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);  
        }


        fL = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        bL = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
        fR = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        bR = (DcMotorEx) hardwareMap.dcMotor.get("backRight");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean alignClaw(ArrayList<Sample> samples) {
        return true;
    }

    public void Transition() {
         
    }

    public void loop_func(Gamepad g, ArrayList<Sample> samples, double RUNTIME) {
        // Clearing the hub bulk cache
        for(LynxModule module: hubs) {
            module.clearBulkCache(); 
        }

        // Update our thingys
        l.update();
        i.update();
        h.update();

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
                        l.clawOpen();
                        // iC is closed, intake at 0, oC is open and oP is at home
                        if(g.right_bumper) {
                            i.clawOpen();
                            i.pitchDown();
                            Intake.pidused = false;
                            ii = INSTATE_INTAKE.GRAB; 
                        }

                        if(g.right_trigger > 0) {
                            i.iM.setPower(0.3);
                        }
                        else if(g.left_trigger > 0) {
                            i.iM.setPower(-0.3);
                        }
                        else {
                            i.iM.setPower(0);
                        }

                        break;
                    }
                    case GRAB: {
                        aligned = alignClaw(samples); 

                        if(aligned && g.left_bumper) {
                            i.clawClose();
                            t=RUNTIME;
                            ii = INSTATE_INTAKE.MOVEBACK;
                        }

                        break; 
                    }
                    case MOVEBACK: {
                        if(t-RUNTIME > 0.3 && Intake.pitchPos == pitchDown) {
                            i.pitchUp();
                            i.cleanYaw();

                            t = RUNTIME;
                        }

                        if(t-RUNTIME > 0.3 && Intake.pitchPos == pitchUp){
                            t = -1;
                        }

                        if(t==-1) {
                            Intake.target = 0;
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
                        l.clawClose();
                        t = -1;
                        it = INSTATE_TRANSFER.TRANSFER2;

                        break;
                    }
                    case TRANSFER2: {   
                        i.clawPartial();

                        if (t == -1) {
                            t = RUNTIME;
                        }   

                        if (t != -1 && RUNTIME - t > 0.2 && g.right_bumper) {
                            t = -1;
                            it = INSTATE_TRANSFER.TRANSFER1;
                            current_claw_state = CLAW_STATE.OUTTAKE;
                            i.clawClose();
                        }
                        break; 
                    }   
                }

                break; 
            }
            case OUTTAKE: {
                switch(io){
                    case LIFT: {
                        l.sample();

                        if(l.oM1.getCurrentPosition() >= 3500) {
                            io = INSTATE_OUTTAKE.TURN;
                        }

                        break; 
                    }
                    case TURN: {
                        l.pitchDrop();

                        if (g.left_bumper) {
                            t = -1;
                            io = INSTATE_OUTTAKE.DROP; 
                        }

                        break;
                    }
                    case DROP: {
                        l.clawOpen();

                        if(t==-1) {
                            t = RUNTIME;
                        }

                        if(t != -1 && (RUNTIME - t > 0.3)) {
                            t = -1;
                            l.clawClose();
                            l.pitchHome();

                            if(g.cross) {
                                l.home();
                                io = INSTATE_OUTTAKE.LIFT;
                                current_claw_state = CLAW_STATE.INTAKE;
                            }
                        }

                        break;
                    }
                }
            }
        }

        if(ENDGAME) {
            if(g.dpad_left) {

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