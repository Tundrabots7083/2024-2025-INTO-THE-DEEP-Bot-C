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
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;

@Config
@TeleOp(name = "Arm isAtTarget Test", group = "tests")
public class ArmIsAtTargetTest extends OpMode {
    public static double ARM_ANGLE = 45.0;
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
            action = robot.arm.actionSetTargetAngle(ARM_ANGLE);
            initialized = true;
        }
        if (run) {
            run = action.run(new TelemetryPacket());
        }
        telemetry.addData("At Target", robot.arm.isAtTarget());
    }

}
