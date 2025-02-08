package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.ArmWithProfile;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlideWithProfile;

@Config
@TeleOp(name = "Arm Profile Slide Test", group = "tests")
public class ArmProfileTest extends OpMode {
    public static double ARM_ANGLE = 0.0;
    public static double LINEAR_SLIDE_LENGTH = 0.0;
    private double lastArmAngle = 0.0;
    private double lastSlideLength = 0.0;
    private ArmWithProfile arm;
    private Robot robot;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot.init(hardwareMap,telemetry);
        robot = Robot.getInstance();

        arm = robot.arm;


        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.intakeAndScoringSubsystem.moveToNeutralPosition();
        arm.setTargetAngle(ARM_ANGLE);
    }

    @Override
    public void loop() {

        if(ARM_ANGLE != lastArmAngle) {
            arm.setTargetAngle(ARM_ANGLE);
            lastArmAngle = ARM_ANGLE;
        }

        if(LINEAR_SLIDE_LENGTH != lastSlideLength) {
            robot.linearSlide.setLength(LINEAR_SLIDE_LENGTH);
            lastSlideLength = LINEAR_SLIDE_LENGTH;
        }

        robot.intakeAndScoringSubsystem.execute();
        telemetry.addData("Target Length", arm.getTargetAngle());
        telemetry.addData("Current Length", arm.getCurrentAngle());
        telemetry.update();

        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        for (LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }
    }

}