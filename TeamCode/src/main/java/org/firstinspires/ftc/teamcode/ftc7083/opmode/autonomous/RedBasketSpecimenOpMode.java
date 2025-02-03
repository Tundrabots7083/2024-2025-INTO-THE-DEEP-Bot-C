package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.RedBasketSample;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.RedBasketSpecimen;

/**
 * Autonomous OpMode used for scoring on the chamber when in the blue alliance.
 */
@Autonomous(name = "Red Basket Specimen", group = "blue", preselectTeleOp = "Primary TeleOp")
@Disabled
public class RedBasketSpecimenOpMode extends AutonomousOpMode {
    @Override
    public Action getTrajectory() {
        Robot robot = Robot.getInstance();
        RedBasketSpecimen trajectoryBuilder = new RedBasketSpecimen(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
        return trajectoryBuilder.getTrajectory();
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(RedBasketSample.INITIAL_POSE_X, RedBasketSample.INITIAL_POSE_Y, RedBasketSample.INITIAL_POSE_ORIENTATION);
    }
}
