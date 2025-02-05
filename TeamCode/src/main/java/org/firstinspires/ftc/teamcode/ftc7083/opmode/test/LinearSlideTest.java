package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlideWithProfile;

@Config
@TeleOp(name = "Linear Slide Test", group = "tests")
public class LinearSlideTest extends OpMode {
    public static double LINEAR_SLIDE_LENGTH = 0.0;
    private double lastLinearSlideLength = 0.0;
    private LinearSlideWithProfile linearSlide;
    private Robot robot;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot.init(hardwareMap,telemetry);
        robot = Robot.getInstance();

        linearSlide = robot.linearSlide;


        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.intakeAndScoringSubsystem.moveToNeutralPosition();
        linearSlide.setLength(LINEAR_SLIDE_LENGTH);
    }

    @Override
    public void loop() {

        if(LINEAR_SLIDE_LENGTH != lastLinearSlideLength) {
            linearSlide.setLength(LINEAR_SLIDE_LENGTH);
            lastLinearSlideLength = LINEAR_SLIDE_LENGTH;
        }

        robot.intakeAndScoringSubsystem.execute();
        robot.arm.setTargetAngle(50);
        telemetry.addData("Target Length", linearSlide.getTargetLength());
        telemetry.addData("Current Length", linearSlide.getCurrentLength());
        telemetry.update();

        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        for (LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }
    }

}