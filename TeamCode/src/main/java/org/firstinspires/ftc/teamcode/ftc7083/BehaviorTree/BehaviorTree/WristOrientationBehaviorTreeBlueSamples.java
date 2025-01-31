package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTree;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.AllianceColor;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.CloseClaw;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.DetectSampleOrientation;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.LowerArmToSubmersibleSample;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.RaiseArmToNeutralPosition;
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
import java.util.List;

public class WristOrientationBehaviorTreeBlueSamples {
    private BehaviorTree tree;
    private Node root;
    private BlackBoardSingleton blackBoard;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected GlobalShutterCamera globalShutterCamera;
    protected Wrist wrist;

    protected IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    private Robot robot;

    public WristOrientationBehaviorTreeBlueSamples(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        Init();
    }

    private void Init() {
        this.blackBoard = BlackBoardSingleton.getInstance(telemetry);
        this.blackBoard.reset();

        robot = Robot.init(hardwareMap,telemetry, Robot.OpModeType.AUTO);
        this.intakeAndScoringSubsystem = robot.intakeAndScoringSubsystem;
        this.globalShutterCamera = robot.globalShutterCamera;
        this.wrist = robot.wrist;

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new DetectSampleOrientation(telemetry, globalShutterCamera, AllianceColor.BLUE),telemetry),
                        new Action(new TurnWrist(telemetry,wrist),telemetry),
                        new Action(new LowerArmToSubmersibleSample(telemetry,intakeAndScoringSubsystem),telemetry),
                        new Action(new CloseClaw(telemetry,robot),telemetry),
                        new Action(new RaiseArmToNeutralPosition(telemetry,intakeAndScoringSubsystem),telemetry)
                ),telemetry);

        this.tree = new BehaviorTree(root, blackBoard);
    }

    public Status tick() {
        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        List<LynxModule> allHubs;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        // Run the behavior tree
        Status result = tree.tick();
       // intakeAndScoringSubsystem.execute();
        telemetry.addData("Wrist Orientation", "Run - Behavior tree result: %s",result);
        telemetry.update();

        return result;
    }
}
