package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.AscentMotor;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;

@Config
@TeleOp(name = "Reset Ascent Motor", group = "tests")
public class ResetAscentMotor extends OpMode {
    private AscentMotor ascentMotor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ascentMotor = new AscentMotor(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
      ascentMotor.setPower(-1);
    }

}
