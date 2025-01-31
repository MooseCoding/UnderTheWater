package org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "barontest")
public class barontest extends OpMode {
    MultipleTelemetry telemetrya = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    Follower f;
    @Override
    public void init() {
        telemetrya.addData("drive coeff", FollowerConstants.drivePIDFCoefficients);
        telemetrya.update();
        Constants.setConstants(FConstants.class, LConstants.class);
        telemetrya.addData("Follower Constants Localizer", FollowerConstants.localizers);
    }
    @Override
    public void start() {
        f = new Follower(hardwareMap);
    }
    @Override
    public void loop() {
        Constants.setConstants(FConstants.class, LConstants.class);
        telemetrya.addData("Follower Constants Localizer", FollowerConstants.localizers);
        telemetrya.addData("drive coeff", FollowerConstants.drivePIDFCoefficients);
        telemetrya.update();
    }
}