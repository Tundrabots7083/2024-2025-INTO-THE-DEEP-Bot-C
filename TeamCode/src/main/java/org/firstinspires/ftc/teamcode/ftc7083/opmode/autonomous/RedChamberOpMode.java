package org.firstinspires.ftc.teamcode.ftc7083.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTree.DetectBlueSamplesBehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTree.DetectRedSamplesBehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.Chamber;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.ChamberWithBarnacleFirst;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.ChamberWithBarnacleNotFirst;

/**
 * Autonomous OpMode used for scoring on the chamber.
 */
@Autonomous(name = "RED Chamber", group = "Active", preselectTeleOp = "Primary TeleOp")
public class RedChamberOpMode extends AutonomousOpMode {
    @Override
    public void Initialize() {
        redBehaviorTree = new DetectRedSamplesBehaviorTree(hardwareMap, telemetry);
    }

    @Override
    public Action getTrajectory() {
        Robot robot = Robot.getInstance();
        if(isFirstMarkBarnacle) {
            ChamberWithBarnacleFirst trajectoryBuilder = new ChamberWithBarnacleFirst(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
            return trajectoryBuilder.getTrajectory();
        } else {
            ChamberWithBarnacleNotFirst trajectoryBuilder = new ChamberWithBarnacleNotFirst(new SparkFunOTOSDrive(hardwareMap, robot.localizer.getPose()));
            return trajectoryBuilder.getTrajectory();
        }
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(Chamber.INITIAL_POSE_X, Chamber.INITIAL_POSE_Y, Chamber.INITIAL_POSE_ORIENTATION);
    }
}
