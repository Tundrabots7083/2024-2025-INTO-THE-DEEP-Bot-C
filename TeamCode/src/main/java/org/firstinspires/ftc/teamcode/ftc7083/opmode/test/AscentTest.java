package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;

@Config
@TeleOp(name = "Ascent Test", group = "tests")
public class AscentTest extends OpMode {
    public static double ARM_ANGLE = Arm.START_ANGLE;
    public static double LINEAR_SLIDE_LENGTH = 0.0;
    private Arm arm;
    private LinearSlide linearSlide;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = new Arm(hardwareMap, telemetry);
        linearSlide = new LinearSlide(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        arm.setTargetAngle(ARM_ANGLE);
        linearSlide.setLength(LINEAR_SLIDE_LENGTH);
        arm.execute();
        linearSlide.execute();
    }

}
