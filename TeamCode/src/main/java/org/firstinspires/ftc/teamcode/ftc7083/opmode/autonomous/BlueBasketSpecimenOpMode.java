package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.action.ParallelAction;
import org.firstinspires.ftc.teamcode.ftc7083.action.SequentialAction;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.BlueBasketSpecimen;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Subsystem;

import java.util.Arrays;
import java.util.List;

/**
 * Autonomous OpMode used for scoring on the chamber when in the blue alliance.
 */
@Autonomous(name = "Blue Basket Specimen", group = "blue extended", preselectTeleOp = "Primary TeleOp")
@Disabled
public class BlueBasketSpecimenOpMode extends OpMode {
    public static int AUTONOMOUS_ACTIONS_TIMEOUT = 27500;
    public static int RETRACT_SLIDE_TIMEOUT = 1000;

    private Robot robot;
    private BlueBasketSpecimen trajectoryBuilder;
    private Action trajectory;
    private List<Subsystem> subsystems;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Robot.INTAKE_COLOR = Robot.SampleIntakeColor.BLUE;

        robot = Robot.init(hardwareMap, telemetry);
        subsystems = Arrays.asList(robot.mecanumDrive, robot.intakeAndScoringSubsystem);
        robot.localizer.setPose(new Pose2d(BlueBasketSpecimen.INITIAL_POSE_X, BlueBasketSpecimen.INITIAL_POSE_Y, BlueBasketSpecimen.INITIAL_ORIENTATION));

        trajectoryBuilder = new BlueBasketSpecimen(new SparkFunOTOSDrive(hardwareMap, new Pose2d(BlueBasketSpecimen.INITIAL_POSE_X, BlueBasketSpecimen.INITIAL_POSE_Y, BlueBasketSpecimen.INITIAL_ORIENTATION)));

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        robot.localizer.update();
    }

    @Override
    public void start() {
        trajectoryBuilder = new BlueBasketSpecimen(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
        trajectory = trajectoryBuilder.getTrajectory();
        robot.intakeAndScoringSubsystem.moveToStartPosition();
    }

    @Override
    public void loop() {
        ActionEx autonomousActions = new ParallelAction(
                (telemetryPacket) -> { // Update all subsystems
                    for (Subsystem subsystem : subsystems) {
                        subsystem.execute();
                    }
                    telemetry.update();
                    return true;
                },
                trajectory
        );
        Action opmodeActions = new SequentialAction(
                autonomousActions.withTimeout(AUTONOMOUS_ACTIONS_TIMEOUT),
                robot.intakeAndScoringSubsystem.actionRetractLinearSlide().withTimeout(RETRACT_SLIDE_TIMEOUT),
                robot.intakeAndScoringSubsystem.actionMoveToStartPosition()
        );
        Actions.runBlocking(opmodeActions);
    }
}
