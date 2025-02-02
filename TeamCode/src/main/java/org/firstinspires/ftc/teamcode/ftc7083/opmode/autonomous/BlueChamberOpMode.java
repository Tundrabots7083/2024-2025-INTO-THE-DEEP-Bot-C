package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.BlueChamber;

/**
 * Autonomous OpMode used for scoring on the chamber when in the blue alliance.
 */
@Autonomous(name = "Blue Chamber", group = "blue", preselectTeleOp = "Primary TeleOp")
public class BlueChamberOpMode extends AutonomousOpMode {
    @Override
    public Action getTrajectory() {
        Robot robot = Robot.getInstance();
        BlueChamber trajectoryBuilder = new BlueChamber(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
        return trajectoryBuilder.getTrajectory();
    }
}
