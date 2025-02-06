package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;

@TeleOp(name = "Ascent Test", group = "tests")
public class AscentTest extends OpMode {
    private Robot robot;
    private final Gamepad previousGamepad1 = new Gamepad();
    private boolean ascend = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = Robot.init(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.cross && !previousGamepad1.cross) {
            ascend = !ascend;
        }

        if (ascend) {
            robot.intakeAndScoringSubsystem.moveToAscentLevelOne();
            telemetry.addData("[ASCENT] pos", "ascent");
        } else {
            robot.intakeAndScoringSubsystem.moveToStartPosition();
            telemetry.addData("[ASCENT] pos", "start");
        }

        robot.arm.execute();
        robot.linearSlide.execute();

        previousGamepad1.copy(gamepad1);
    }

}
