package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTree;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.AllianceColor;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.CloseClaw;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.DetectSampleOrientation;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.DetectYellowSamples;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.ExtendArmToSubmersibleSample;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.IsBotOriented;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.LowerArmToSubmersibleSample;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.MoveToStartPosition;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.NavigateAndOrientToSample;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.OpenClaw;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.RaiseArmToNeutralPosition;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.TurnWrist;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions.WristIntakePosition;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Conditional;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Selector;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.GlobalShutterCamera;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Limelight;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

import java.util.Arrays;
import java.util.List;

public class IntakeRedSampleBehaviorTree {

    private BehaviorTree tree;
    private Node root;
    private BlackBoardSingleton blackBoard;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected Limelight limelight;
    protected GlobalShutterCamera globalShutterCamera;
    protected Wrist wrist;
    protected IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    protected MecanumDrive mecanumDrive;
    private Robot robot;

    public IntakeRedSampleBehaviorTree(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        Init();
    }

    private void Init() {
        this.blackBoard = BlackBoardSingleton.getInstance(telemetry);
        this.blackBoard.reset();
        this.limelight = new Limelight(hardwareMap,telemetry);
        this.globalShutterCamera = new GlobalShutterCamera(hardwareMap, telemetry);

        telemetry.addLine("Before Robot");
        telemetry.update();
        robot = Robot.init(hardwareMap,telemetry, Robot.OpModeType.AUTO);
        this.intakeAndScoringSubsystem = robot.intakeAndScoringSubsystem;
        this.wrist = robot.wrist;
        telemetry.addLine("Got Past Robot");
        telemetry.update();

        this.mecanumDrive = new MecanumDrive(hardwareMap,telemetry);

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new MoveToStartPosition(telemetry,intakeAndScoringSubsystem),telemetry),
                        new Action(new OpenClaw(telemetry,robot),telemetry),
                        new Selector(
                                Arrays.asList(
                                        new Conditional(new IsBotOriented()),
                                        new Action(new DetectYellowSamples(telemetry,limelight, Limelight.TargetPosition.SUBMERSIBLE),telemetry)),telemetry
                        ),
                        new Action(new NavigateAndOrientToSample(telemetry,mecanumDrive),telemetry),
                        new Action(new RaiseArmToNeutralPosition(telemetry,intakeAndScoringSubsystem),telemetry),
                        new Action(new WristIntakePosition(telemetry,wrist),telemetry),
                        new Action(new ExtendArmToSubmersibleSample(telemetry,intakeAndScoringSubsystem),telemetry),
                        new Action(new DetectSampleOrientation(telemetry, globalShutterCamera, AllianceColor.RED),telemetry),
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
        intakeAndScoringSubsystem.execute();
        telemetry.addData("IntakeSample", "Run - Behavior tree result: %s",result);
        telemetry.update();

        return result;
    }
}
