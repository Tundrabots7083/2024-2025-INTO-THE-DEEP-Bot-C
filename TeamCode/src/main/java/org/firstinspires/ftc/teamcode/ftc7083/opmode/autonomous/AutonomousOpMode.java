package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionExBase;
import org.firstinspires.ftc.teamcode.ftc7083.action.ParallelAction;
import org.firstinspires.ftc.teamcode.ftc7083.action.SequentialAction;
import org.firstinspires.ftc.teamcode.ftc7083.action.TimeoutAction;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Subsystem;

import java.util.Arrays;
import java.util.List;

/**
 * Base autonomous op mode
 */
public abstract class AutonomousOpMode extends LinearOpMode {
    public static int AUTONOMOUS_ACTIONS_TIMEOUT = 27000;
    public static int RETRACT_SLIDE_TIMEOUT = 1500;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the op-mode
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Robot.INTAKE_COLOR = Robot.SampleIntakeColor.BLUE;

        Robot robot = Robot.init(hardwareMap, telemetry);

        List<Subsystem> subsystems = Arrays.asList(robot.mecanumDrive, robot.intakeAndScoringSubsystem);
        robot.localizer.setPose(getInitialPose());

        // Run the trajectory and, once done, move to the start position, all the while ensuring the
        // subsystems are updated
        Action autonomousActions = new ParallelAction(
                (telemetryPacket) -> { // Update all subsystems
                    for (Subsystem subsystem : subsystems) {
                        subsystem.execute();
                    }
                    telemetry.update();
                    return true;
                },
                new SequentialAction(
                        new TimeoutAction(getTrajectory(), AUTONOMOUS_ACTIONS_TIMEOUT),
                        robot.intakeAndScoringSubsystem.actionRetractLinearSlide().withTimeout(RETRACT_SLIDE_TIMEOUT),
                        robot.intakeAndScoringSubsystem.actionMoveToStartPosition()
                )
        );

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        // Run during init but before the start button is pressed
        while (opModeInInit()) {
            robot.localizer.update();
        }

        waitForStart();

        // Handle the stop button being pressed immediately after the start button has been pressed
        if (isStopRequested()) return;

        // Move the robot's intake and scoring subsystem from the ground
        robot.intakeAndScoringSubsystem.moveToStartPosition();

        // Run the autonomous actions
        Actions.runBlocking(autonomousActions);
    }

    /**
     * Gets the initial pose for the trajectory.
     *
     * @return the initial pose for the trajectory
     */
    public abstract Pose2d getInitialPose();

    /**
     * Gets the trajectory to run in the autonomous OpMode.
     *
     * @return the trajectory to run in the autonomous OpMode
     */
    public abstract Action getTrajectory();
}
