package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;

/**
 * Base autonomous op mode
 */
@Config
@Autonomous(name = "Autonomous Trajectory Test", group = "tests")
public class AutonomousTest extends LinearOpMode {
    // Initial robot pose
    public static double INITIAL_POSE_X = 0.0;
    public static double INITIAL_POSE_Y = 0.0;
    public static double INITIAL_POSE_ORIENTATION = 90.0;

    // Drive constants
    public static double DRIVE_Y_DISTANCE = 48.0; // OTOS and RoadRunner report 48 when driven by RoadRunner
    public static double DRIVE_X_DISTANCE = 12.0;
    public static double DRIVE_ORIENTATION = 90.0;

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

        Pose2d otosPose = robot.localizer.getPose();
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, otosPose);

        Pose2d robotPose = new Pose2d(new Vector2d(INITIAL_POSE_X, INITIAL_POSE_Y), Math.toRadians(INITIAL_POSE_ORIENTATION));
        Action driveActions = drive.actionBuilder(robotPose)
//                .lineToY(DRIVE_Y_DISTANCE)
                .strafeToSplineHeading(new Vector2d(DRIVE_X_DISTANCE, 2), Math.toRadians(INITIAL_POSE_ORIENTATION))
                .build();
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("OTOS Pose", otosPose);
        telemetry.addData("Robot Pose", robotPose);
        telemetry.update();

        // Run the action, making sure to update the telemetry as the actions run
        Actions.runBlocking(
                new ParallelAction(
                        driveActions,
                        new ParallelAction(
                                (telemetryPacket) -> {
                                    telemetry.update();
                                    return true;
                                },
                                driveActions
                        )
                )
        );
    }
}
