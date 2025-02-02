package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.action.ParallelAction;
import org.firstinspires.ftc.teamcode.ftc7083.action.SequentialAction;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.RedChamber;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Subsystem;

import java.util.Arrays;
import java.util.List;

/**
 * Autonomous OpMode used for scoring on the chamber when in the blue alliance.
 */
@Autonomous(name = "Red Chamber", group = "red", preselectTeleOp = "Primary TeleOp")
public class RedChamberOpMode extends LinearOpMode {
    public static int AUTONOMOUS_ACTIONS_TIMEOUT = 27500;
    public static int RETRACT_SLIDE_TIMEOUT = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the op-mode
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Robot.INTAKE_COLOR = Robot.SampleIntakeColor.BLUE;

        Robot robot = Robot.init(hardwareMap, telemetry);

        List<Subsystem> subsystems = Arrays.asList(robot.mecanumDrive, robot.intakeAndScoringSubsystem);
        robot.localizer.setPose(new Pose2d(RedChamber.INITIAL_POSE_X, RedChamber.INITIAL_POSE_Y, RedChamber.INITIAL_POSE_ORIENTATION));

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        // Run during init but before the start button is pressed
        while (opModeInInit()) {
            robot.localizer.update();
        }

        waitForStart();

        // Prepare the trajectories once the start button is pressed
        RedChamber trajectoryBuilder = new RedChamber(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
        Action trajectory = trajectoryBuilder.getTrajectory();

        // Handle the stop button being pressed immediately after the start button has been pressed
        if (isStopRequested()) return;

        robot.intakeAndScoringSubsystem.moveToStartPosition();

        // Run the trajectory
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
        Actions.runBlocking(autonomousActions);
    }
}
