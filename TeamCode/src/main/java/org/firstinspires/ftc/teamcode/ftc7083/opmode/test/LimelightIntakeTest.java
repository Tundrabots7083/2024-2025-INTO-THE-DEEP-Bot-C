package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;


@TeleOp(name = "Limelight Arm Extension Test", group = "test")
public class LimelightIntakeTest extends OpMode {
    private final Gamepad currentGamepad1 = new Gamepad();
    private org.firstinspires.ftc.teamcode.ftc7083.Robot robot;
    private IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    private MecanumDrive mecanumDrive;


    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot = Robot.init(hardwareMap, telemetry);
        intakeAndScoringSubsystem = new IntakeAndScoringSubsystem(hardwareMap,telemetry);
        mecanumDrive = new MecanumDrive(hardwareMap,telemetry);

        telemetry.setMsTransmissionInterval(11);
        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.intakeAndScoringSubsystem.moveToNeutralPosition();
    }

    @Override
    public void loop() {
        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        for (LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }

        // Always get a copy of the current gamepads. The gamepads are updated in near real time,
        // so this ensures that a consistent set of values are used within each subsystem controller.
        currentGamepad1.copy(gamepad1);

        // Process gamepad controller inputs
        if (gamepad1.cross) {
//            intakeAndScoringSubsystem.moveToSampleIntakePosition();
        }

        robot.arm.execute();
        robot.linearSlide.execute();
//        robot.limelight.execute();
        telemetry.update();
    }


}
