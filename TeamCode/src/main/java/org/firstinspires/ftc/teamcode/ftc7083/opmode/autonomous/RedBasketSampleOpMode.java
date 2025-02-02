package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.RedBasketSample;

/**
 * Autonomous OpMode used for scoring on the chamber when in the blue alliance.
 */
@Autonomous(name = "Red Basket", group = "blue", preselectTeleOp = "Primary TeleOp")
public class RedBasketSampleOpMode extends AutonomousOpMode {
    @Override
    public Action getTrajectory() {
        Robot robot = Robot.getInstance();
        RedBasketSample trajectoryBuilder = new RedBasketSample(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
        return trajectoryBuilder.getTrajectory();
    }
}
