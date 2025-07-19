package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTree;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.DetectBlueSamplesSimplied;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.DetectRedSamplesSimplied;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Limelight;

import java.util.Arrays;

public class DetectRedSamplesBehaviorTree {
    private BehaviorTree tree;
    private Node root;
    public BlackBoardSingleton blackBoard;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected Limelight limelight;

    protected IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    private Robot robot;

    public DetectRedSamplesBehaviorTree(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        Init();
    }

    private void Init() {
        this.blackBoard = BlackBoardSingleton.getInstance(telemetry);
        this.blackBoard.reset();

        robot = Robot.getInstance();
        this.intakeAndScoringSubsystem = robot.intakeAndScoringSubsystem;
        this.limelight = robot.limelight;

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new DetectRedSamplesSimplied(telemetry, limelight),telemetry)
                ),telemetry);

        this.tree = new BehaviorTree(root, blackBoard);
    }

    public Status tick() {
        // Run the behavior tree
        Status result = tree.tick();
        telemetry.addData("Wrist Orientation", "Run - Behavior tree result: %s",result);
        telemetry.update();

        return result;
    }
}
