package org.firstinspires.ftc.teamcode.ftc7083.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.math.FTCMath;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller.IntakeAndScoringSubsystemController;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller.MecanumDriveController;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller.SubsystemController;

import java.util.Arrays;
import java.util.Collection;

/**
 * Main OpMode used by a driver for the OUT_OF_THE_BLUE game.
 */
@Config
@TeleOp(name = "Primary TeleOp", group = "Active")
public class PrimaryTeleOp extends OpMode {
    public static boolean RAN_AUTONOMOUS = false;
    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private Robot robot;
    private Collection<SubsystemController> controllers;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot = Robot.init(hardwareMap, telemetry);

        MecanumDriveController mecanumDriveController = new MecanumDriveController(robot.mecanumDrive, telemetry);
        IntakeAndScoringSubsystemController intakeAndScoringSubsystemController = new IntakeAndScoringSubsystemController(robot.intakeAndScoringSubsystem, telemetry);
        controllers = Arrays.asList(mecanumDriveController, intakeAndScoringSubsystemController);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.intakeAndScoringSubsystem.moveToStartPosition();
    }

    @Override
    public void loop() {
        timer.reset();

        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        for (LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }

        // Always get a copy of the current gamepads. The gamepads are updated in near real time,
        // so this ensures that a consistent set of values are used within each subsystem controller.
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // Process gamepad controller inputs
        for (SubsystemController controller : controllers) {
            controller.execute(currentGamepad1, currentGamepad2);
        }

        // Update the location of the robot on the field using April Tag localization
        robot.localizer.update();

        // OTOS telemetry
        SparkFunOTOS.Pose2D position = robot.otos.getPosition();
        telemetry.addData("[OTOS] x", position.x);
        telemetry.addData("[OTOS] y", position.y);
        telemetry.addData("[OTOS] h", position.h);

        // Print out the loop time, in milliseconds, to two decimal places
        telemetry.addData("Loop Time", FTCMath.round(timer.time(), 2));

        telemetry.update();
    }
}
