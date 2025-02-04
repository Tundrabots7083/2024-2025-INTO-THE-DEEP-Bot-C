package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.BlueBasketSample;

/**
 * Autonomous OpMode used for scoring on the chamber when in the blue alliance.
 */
@Autonomous(name = "Blue Basket", group = "blue", preselectTeleOp = "Primary TeleOp")
public class BlueBasketSampleOpMode extends AutonomousOpMode {
    @Override
    public Action getTrajectory() {
        Robot.INTAKE_COLOR = Robot.SampleIntakeColor.BLUE;
        Robot robot = Robot.getInstance();
        BlueBasketSample trajectoryBuilder = new BlueBasketSample(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
        return trajectoryBuilder.getTrajectory();
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(BlueBasketSample.INITIAL_POSE_X, BlueBasketSample.INITIAL_POSE_Y, BlueBasketSample.INITIAL_POSE_ORIENTATION);
    }
}
