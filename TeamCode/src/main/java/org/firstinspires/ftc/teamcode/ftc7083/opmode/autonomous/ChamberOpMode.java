package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.Chamber;

/**
 * Autonomous OpMode used for scoring on the chamber.
 */
@Autonomous(name = "Chamber", group = "red", preselectTeleOp = "Primary TeleOp")
public class ChamberOpMode extends AutonomousOpMode {
    @Override
    public Action getTrajectory() {
        Robot robot = Robot.getInstance();
        Chamber trajectoryBuilder = new Chamber(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
        return trajectoryBuilder.getTrajectory();
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(Chamber.INITIAL_POSE_X, Chamber.INITIAL_POSE_Y, Chamber.INITIAL_POSE_ORIENTATION);
    }
}
