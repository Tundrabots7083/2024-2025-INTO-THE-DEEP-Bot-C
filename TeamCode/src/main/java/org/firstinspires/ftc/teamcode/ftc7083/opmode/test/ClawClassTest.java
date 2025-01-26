package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Claw;

@Config
@TeleOp(name = "Claw Class Test", group = "tests")
public class ClawClassTest extends OpMode {
    private Claw claw;
    private boolean clawOpened = false;
    private final Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        claw = new Claw(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.cross && !previousGamepad1.cross) {
            if (clawOpened) {
                claw.close();
                clawOpened = false;
            } else {
                claw.open();
                clawOpened = true;
            }
        }

        telemetry.addData("Claw opened", clawOpened);

        telemetry.addData("Retrieved degrees", claw.getCurrentDegrees());
        telemetry.update();

        previousGamepad1.copy(gamepad1);
    }
}
