package org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTree;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions.DetectSampleOrientation;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions.ResetWrist;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions.TurnWrist;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.GlobalShutterCamera;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

import java.util.Arrays;
import java.util.List;

public class WristOrientationBehaviorTree {
    private BehaviorTree tree;
    private Node root;
    private BlackBoardSingleton blackBoard;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected GlobalShutterCamera globalShutterCamera;
    protected Wrist wrist;

    protected IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    private Robot robot;

    public WristOrientationBehaviorTree(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        Init();
    }

    private void Init() {
        this.blackBoard = BlackBoardSingleton.getInstance(telemetry);
        this.globalShutterCamera = new GlobalShutterCamera(hardwareMap, telemetry);
       this.wrist = new Wrist(hardwareMap, telemetry);


        this.intakeAndScoringSubsystem = new IntakeAndScoringSubsystem(hardwareMap,telemetry);
       // robot = Robot.init(hardwareMap,telemetry, Robot.OpModeType.AUTO);

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new ResetWrist(telemetry,wrist),telemetry),
                        new Action(new DetectSampleOrientation(telemetry, globalShutterCamera),telemetry),
                        new Action(new TurnWrist(telemetry,wrist),telemetry)
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
