package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.ArmWithProfile;

@Config
@TeleOp(name = "Arm Test", group = "tests")
public class ArmTest extends OpMode {
    public static double ARM_ANGLE = ArmWithProfile.START_ANGLE;
    private ArmWithProfile arm;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = new ArmWithProfile(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        arm.setTargetAngle(ARM_ANGLE);
        arm.execute();
    }

}
