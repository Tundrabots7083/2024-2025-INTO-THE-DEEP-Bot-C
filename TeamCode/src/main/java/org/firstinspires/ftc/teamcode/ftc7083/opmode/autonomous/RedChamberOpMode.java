package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.RedChamber;

/**
 * Autonomous OpMode used for scoring on the chamber when in the blue alliance.
 */
@Autonomous(name = "Red Chamber", group = "blue", preselectTeleOp = "Primary TeleOp")
public class RedChamberOpMode extends AutonomousOpMode {
    @Override
    public Action getTrajectory() {
        Robot.INTAKE_COLOR = Robot.SampleIntakeColor.RED;
        Robot robot = Robot.getInstance();
        RedChamber trajectoryBuilder = new RedChamber(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
        return trajectoryBuilder.getTrajectory();
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(RedChamber.INITIAL_POSE_X, RedChamber.INITIAL_POSE_Y, RedChamber.INITIAL_POSE_ORIENTATION);
    }
}
