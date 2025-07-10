package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.AscentMotor;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;

@TeleOp(name = "Ascent Test", group = "tests")
public class AscentTest extends OpMode {
    private Robot robot;
    private Arm arm;
    private LinearSlide linearSlide;
    private AscentMotor ascentMotor;
    private final Gamepad previousGamepad1 = new Gamepad();
    private int buttonCount = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = Robot.init(hardwareMap, telemetry);
        arm = robot.arm;
        linearSlide = robot.linearSlide;
        ascentMotor = new AscentMotor(linearSlide, hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.cross && !previousGamepad1.cross) {
            buttonCount++;
        }

        switch (buttonCount) {
            case 1:
                arm.setTargetAngle(50);
                linearSlide.setLength(15);
                telemetry.addData("[ASCENT] pos", "ascent");
                break;
            case 2:
                arm.setTargetAngle(-20);
                break;
            case 3:
                linearSlide.setSlideToZeroPower();
                ascentMotor.ascend();
                buttonCount++;
                break;
            case 4:
                if (ascentMotor.isAtTarget()) {
                    gamepad1.rumble(500);
                }
                break;


        }

        linearSlide.isAscending = ascentMotor.isAscending;

        robot.arm.execute();
        robot.linearSlide.execute();
        ascentMotor.execute();

        previousGamepad1.copy(gamepad1);
    }

}
