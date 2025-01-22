package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;

@TeleOp(name = "LinearSlide isAtTarget Test", group = "tests")
public class LinearSlideIsAtTargetTest extends OpMode {
    private Robot robot;
    private boolean initialized = false;
    private boolean run = true;
    private ActionEx action;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = Robot.init(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        for (LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }
        if (!initialized) {
            action = new IntakeAndScoringSubsystem.MoveToChamberHighScoringPosition(robot.intakeAndScoringSubsystem);
            initialized = true;
        }
        if (run) {
            run = action.run(new TelemetryPacket());
        }
        telemetry.addData("At Target", robot.linearSlide.isAtTarget());
    }
}
