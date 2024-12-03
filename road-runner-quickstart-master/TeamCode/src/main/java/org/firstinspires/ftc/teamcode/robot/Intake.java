package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    public DcMotorEx iM;
    private PIDController controller;
    private Servo intakeClaw, intakePitch, intakeYaw;

    public static double target = 0.0;

    public static boolean pidused = true;

    public static double pitchDown = 1; // Done
    public static double pitchUp = 0.25; // Done

    public static double clawClose = 0.3; // Done
    public static double clawOpen = 0.7; // Done
    public static double cleanYaw = 0.39; // Done

    public static double clawPartial = 0.4;

    public static double pitchPos = pitchUp, clawPos = clawClose, yawPos = cleanYaw;


    public Intake(HardwareMap hardwareMap) {
        iM = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        iM.setDirection(DcMotorSimple.Direction.REVERSE);
        double p = 0.02;
        double i = 0.000001;
        double d = 0.0003;
        controller = new PIDController(p,i,d);
        intakeClaw = hardwareMap.servo.get("intakeClaw");
        intakeYaw = hardwareMap.servo.get("intakeYaw");
        intakePitch = hardwareMap.servo.get("intakePitch");
    }

    public void update() {
        if(pidused) {
            iM.setPower(controller.calculate(iM.getCurrentPosition(), target));
        }

        intakePitch.setPosition(pitchPos);
        intakeClaw.setPosition(clawPos);
        intakeYaw.setPosition(yawPos);
    }

    public void pitchDown() {
        pitchPos = pitchDown;
    }

    public void pitchUp() {
        pitchPos = pitchUp;
    }

    public void clawClose() {
        clawPos = clawClose;
    }

    public void clawOpen() {
        clawPos = clawOpen;
    }

    public void incrementYaw(double increment) {
        intakeYaw.setPosition(yawPos + increment);
        yawPos += increment;
    }


    public void clawPartial() {
        clawPos = clawPartial;
    }

    public void cleanYaw() {
        yawPos = cleanYaw;
    }

    public class IntakeClawClose implements Action {
        private boolean init = false;
        private Intake i;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                i.clawOpen();
                init = true;
            }

            return false;
        }

        public IntakeClawClose(Intake li) {
            i = li;
        }

        public Action intakeClawClose(Intake li) {
            return new IntakeClawClose(li);
        }
    }


    public class IntakeClawOpen implements Action {
        private boolean init = false;
        private Intake i;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                i.clawOpen();
                init = true;
            }

            return false;
        }

        public IntakeClawOpen(Intake li) {
            i = li;
        }

        public Action intakeClawOpen(Intake li) {
            return new IntakeClawOpen(li);
        }
    }


    public class IntakeIn implements Action {
        private boolean init = false;
        private Intake i;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                Intake.target = 0;
                init = true;
            }

            return i.iM.getCurrentPosition() > 40;
        }

        public IntakeIn(Intake li) {
            i = li;
        }

        public Action intakeIn(Intake li) {
            return new IntakeIn(li);
        }
    }

    public class IntakeOut implements Action {
        private boolean init = false;
        private Intake i;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                Intake.target = 200;
                init = true;
            }

            return i.iM.getCurrentPosition() < 150;
        }

        public IntakeOut(Intake li) {
            i = li;
        }

        public Action intakeOut(Intake li) {
            return new IntakeOut(li);
        }
    }

    public class IntakePitchDown implements Action {
        private boolean init = false;
        private Intake i;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                i.pitchDown();
                init = true;
            }

            return false;
        }

        public IntakePitchDown(Intake li) {
            i = li;
        }

        public Action intakePitchDown(Intake li) {
            return new IntakePitchDown(li);
        }
    }


    public class IntakePitchUp implements Action {
        private boolean init = false;
        private Intake i;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                i.pitchUp();
                init = true;
            }

            return false;
        }

        public IntakePitchUp(Intake li) {
            i = li;
        }

        public Action intakePitchUp(Intake li) {
            return new IntakePitchUp(li);
        }
    }
}
