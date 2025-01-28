package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.RedChamber;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Subsystem;

import java.util.Arrays;
import java.util.List;

/**
 * Autonomous OpMode used for scoring on the chamber when in the blue alliance.
 */
@Autonomous(name = "Red Chamber", group = "Active", preselectTeleOp = "Primary TeleOp")
public class RedChamberOpMode extends OpMode {
    private Robot robot;
    private RedChamber trajectoryBuilder;
    private Action trajectory;
    private List<Subsystem> subsystems;
    private boolean actionsRunning = true;
    private Canvas canvas;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot = Robot.init(hardwareMap, telemetry);
        subsystems = Arrays.asList(robot.mecanumDrive, robot.arm, robot.linearSlide, robot.claw, robot.wrist);
        robot.localizer.setPose(new Pose2d(RedChamber.INITIAL_POSE_X, RedChamber.INITIAL_POSE_Y, RedChamber.INITIAL_HEADING));

        trajectoryBuilder = new RedChamber(new SparkFunOTOSDrive(hardwareMap, new Pose2d(RedChamber.INITIAL_POSE_X, RedChamber.INITIAL_POSE_Y, RedChamber.INITIAL_HEADING)));

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        robot.localizer.update();
    }

    @Override
    public void start() {
        trajectoryBuilder = new RedChamber(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
        trajectory = trajectoryBuilder.getTrajectory();
        canvas = new Canvas();
        trajectory.preview(canvas);
        robot.intakeAndScoringSubsystem.moveToStartPosition();
    }

    @Override
    public void loop() {
        // Update all the hardware subsystems and the localizer
        for (Subsystem subsystem : subsystems) {
            subsystem.execute();
        }

        // Run the trajectory action. We aren't using Actions.runBlocking so that we can make sure
        // our subsystems continue to be given a chance to execute.
        if (actionsRunning) {
            TelemetryPacket tp = new TelemetryPacket();
            tp.fieldOverlay().getOperations().addAll(canvas.getOperations());
            actionsRunning = trajectory.run(tp);
            FtcDashboard.getInstance().sendTelemetryPacket(tp);
        }

        telemetry.update();
    }
}
