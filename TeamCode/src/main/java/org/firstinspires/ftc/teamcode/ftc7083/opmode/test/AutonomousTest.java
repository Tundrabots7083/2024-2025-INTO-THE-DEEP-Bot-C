package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;

/**
 * Base autonomous op mode
 */
@Config
public class AutonomousTest extends LinearOpMode {
    public static double INITIAL_POSE_X = 0.0;
    public static double INITIAL_POSE_Y = 0.0;
    public static double INITIAL_POSE_ORIENTATION = 90.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the op-mode
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Robot robot = Robot.init(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        // Run during init but before the start button is pressed
        while (opModeInInit()) {
            robot.localizer.update();
            telemetry.update();
        }
        
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose());
        Action driveActions = drive.actionBuilder(new Pose2d(new Vector2d(INITIAL_POSE_X, INITIAL_POSE_Y), Math.toRadians(INITIAL_POSE_ORIENTATION)))
                .lineToY(48)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(driveActions);
    }
}
