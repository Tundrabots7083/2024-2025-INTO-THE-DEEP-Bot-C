package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.action.ParallelAction;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.RedBasketSpecimen;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Subsystem;

import java.util.Arrays;
import java.util.List;

/**
 * Autonomous OpMode used for scoring on the chamber when in the blue alliance.
 */
@Autonomous(name = "Red Basket Specimen", group = "Active", preselectTeleOp = "Primary TeleOp")
public class RedBasketSpecimenOpMode extends OpMode {
    private Robot robot;
    private RedBasketSpecimen trajectoryBuilder;
    private Action trajectory;
    private List<Subsystem> subsystems;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot = Robot.init(hardwareMap, telemetry);
        subsystems = Arrays.asList(robot.mecanumDrive, robot.intakeAndScoringSubsystem);
        robot.localizer.setPose(new Pose2d(RedBasketSpecimen.INITIAL_POSE_X, RedBasketSpecimen.INITIAL_POSE_Y, RedBasketSpecimen.INITIAL_ORIENTATION));

        trajectoryBuilder = new RedBasketSpecimen(new SparkFunOTOSDrive(hardwareMap, new Pose2d(RedBasketSpecimen.INITIAL_POSE_X, RedBasketSpecimen.INITIAL_POSE_Y, RedBasketSpecimen.INITIAL_ORIENTATION)));

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        robot.localizer.update();
    }

    @Override
    public void start() {
        trajectoryBuilder = new RedBasketSpecimen(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
        trajectory = trajectoryBuilder.getTrajectory();
        robot.intakeAndScoringSubsystem.moveToStartPosition();
    }

    @Override
    public void loop() {
        Action autonomousActions = new ParallelAction(
                (telemetryPacket) -> { // Update all subsystems
                    for (Subsystem subsystem : subsystems) {
                        subsystem.execute();
                    }
                    telemetry.update();
                    return true;
                },
                trajectory
        );
        Actions.runBlocking(autonomousActions);
    }
}
