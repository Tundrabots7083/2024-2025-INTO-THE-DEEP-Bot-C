package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.AscentMotor;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;

@Config
@TeleOp(name = "Ascent Motor Test", group = "tests")
public class AscentMotorTest extends OpMode {
    public static double LINEAR_SLIDE_LENGTH = 0.0;
    private double lastLinearSlideLength = 0.0;
    private AscentMotor ascentMotor;
    private LinearSlide linearSlide;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        linearSlide = new LinearSlide(hardwareMap, telemetry);
        ascentMotor = new AscentMotor(linearSlide, hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {

        if(LINEAR_SLIDE_LENGTH != lastLinearSlideLength) {
            linearSlide.setSlideToZeroPower();
            lastLinearSlideLength = LINEAR_SLIDE_LENGTH;
        }

        ascentMotor.execute();
        telemetry.addData("Current Length", ascentMotor.getCurrentLinearSlideLength());
        telemetry.update();
    }

}
