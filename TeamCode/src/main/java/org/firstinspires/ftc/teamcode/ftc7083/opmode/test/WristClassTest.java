package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

@Config
@TeleOp(name = "Wrist Class Test", group = "tests")
public class WristClassTest extends OpMode {
    public static double PITCH_DEGREES = 0.0;
    public static double ROLL_DEGREES = 0.0;

    private Wrist wrist;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        wrist = new Wrist(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        wrist.setPitchDegrees(PITCH_DEGREES);
        wrist.setRollDegrees(ROLL_DEGREES);

        telemetry.addData("Target pitchDegrees", PITCH_DEGREES);
        telemetry.addData("Target rollDegrees", ROLL_DEGREES);
        telemetry.addData("Retrieved pitchDegrees", wrist.getPitchDegrees());
        telemetry.addData("Retrieved rollDegrees", wrist.getRollDegrees());
        telemetry.addData("Retrieved pitchPosition", wrist.getPitchPosition());
        telemetry.addData("Retrieved rollPosition", wrist.getRollPosition());
        telemetry.addData("At Target", wrist.isAtTarget());
        telemetry.update();
    }
}
