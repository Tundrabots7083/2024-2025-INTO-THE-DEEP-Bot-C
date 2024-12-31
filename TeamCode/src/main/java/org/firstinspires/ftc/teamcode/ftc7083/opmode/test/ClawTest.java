package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import static org.firstinspires.ftc.teamcode.ftc7083.subsystem.Claw.DEFAULT_OPEN_DEGREES;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Claw;

/**
 * This OpMode tests the claw using FTC Dashboard (<a href="http://192.168.43.1:8080/dash">...</a>)
 */
@Config
@TeleOp(name = "Claw Test TeleOp", group = "tests")
public class ClawTest extends OpMode {
    // Specifies the number of degrees the claw is to be opened to
    public static double CLAW_OPEN_DEGREES = 0.0;
    // When true causes claw to be opened to the claw's default open degrees
    public static boolean DEFAULT_OPEN = false;
    // When true causes the claw to be opened to the degrees value in CLAW_OPEN_DEGREES
    public static boolean OPEN_WITH_DEGREES = false;
    // When true causes the claw to be closed to the 0 degrees position
    public static boolean CLOSE = false;
    Claw claw;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        claw = new Claw(hardwareMap, telemetry);
        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        double currentClawPosition; // Current position of claw in degrees
        if (DEFAULT_OPEN) {
            telemetry.addData("Open claw to default degrees", DEFAULT_OPEN_DEGREES);
            claw.open();
            currentClawPosition = claw.getCurrentPosition();
            telemetry.addData("Claw position after open claw to default", currentClawPosition);
        } else if (OPEN_WITH_DEGREES) {
            telemetry.addData("Open claw to specified degrees", CLAW_OPEN_DEGREES);
            claw.setDegrees(CLAW_OPEN_DEGREES);
            currentClawPosition = claw.getCurrentPosition();
            telemetry.addData("Claw position after open claw to specified degrees", currentClawPosition);
        } else if (CLOSE) {
            telemetry.addLine("Close claw");
            claw.close();
            currentClawPosition = claw.getCurrentPosition();
            telemetry.addData("Current position after close", currentClawPosition);
        }
        telemetry.update();
    }
}
