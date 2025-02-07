package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTree;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.DetectAprilTags_GlobalShutter;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.DetectSampleOrientation;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.NavigateAndOrientToChamber;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.TurnWrist;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.GlobalShutterCamera;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

import java.util.Arrays;

public class OrientationToChamberBehaviorTree {
    private BehaviorTree tree;
    private Node root;
    private BlackBoardSingleton blackBoard;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected GlobalShutterCamera globalShutterCamera;
    protected Wrist wrist;

    protected IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    private Robot robot;

    public OrientationToChamberBehaviorTree(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        Init();
    }

    private void Init() {
        this.blackBoard = BlackBoardSingleton.getInstance(telemetry);
        this.blackBoard.reset();

        robot = Robot.getInstance();
        this.intakeAndScoringSubsystem = robot.intakeAndScoringSubsystem;
        this.globalShutterCamera = robot.wristGlobalShutterCamera;
        this.wrist = robot.wrist;

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new DetectAprilTags_GlobalShutter(telemetry,robot),telemetry),
                        new Action(new NavigateAndOrientToChamber(telemetry,robot.mecanumDrive),telemetry)
                ),telemetry);

        this.tree = new BehaviorTree(root, blackBoard);
    }

    public Status tick() {

        // Run the behavior tree
        Status result = tree.tick();
        //intakeAndScoringSubsystem.execute();
        telemetry.addData("Wrist Orientation", "Run - Behavior tree result: %s",result);
        telemetry.update();

        return result;
    }
}
