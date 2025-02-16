package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.Basket;

/**
 * Autonomous OpMode used for scoring on the basket.
 */
@Autonomous(name = "Basket", group = "red", preselectTeleOp = "Primary TeleOp")
public class BasketOpMode extends AutonomousOpMode {
    @Override
    public Action getTrajectory() {
        Robot robot = Robot.getInstance();
        Basket trajectoryBuilder = new Basket(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
        return trajectoryBuilder.getTrajectory();
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(Basket.INITIAL_POSE_X, Basket.INITIAL_POSE_Y, Basket.INITIAL_POSE_ORIENTATION);
    }
}
