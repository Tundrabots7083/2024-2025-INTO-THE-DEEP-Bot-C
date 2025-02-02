package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
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
        Robot robot = Robot.getInstance();
        BlueBasketSample trajectoryBuilder = new BlueBasketSample(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
        return trajectoryBuilder.getTrajectory();
    }
}
