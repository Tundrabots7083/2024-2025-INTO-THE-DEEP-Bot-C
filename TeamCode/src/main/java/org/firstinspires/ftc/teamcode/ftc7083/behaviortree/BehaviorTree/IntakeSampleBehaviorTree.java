package org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTree;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions.CloseClaw;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions.DetectYellowSamples;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions.ExtendArmToSubmersibleSample;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions.IsBotOriented;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions.NavigateWithinRangeOfSample;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions.OpenClaw;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions.RaiseArmToNeutralPosition;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions.SquareBotWithSample;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Conditional;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Selector;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Limelight;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;

import java.util.Arrays;

public class IntakeSampleBehaviorTree {

    private BehaviorTree tree;
    private Node root;
    private BlackBoardSingleton blackBoard;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected Limelight limelight;
    protected IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    protected MecanumDrive mecanumDrive;
    private Robot robot;

    public IntakeSampleBehaviorTree(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        Init();
    }

    private void Init() {
        this.blackBoard = BlackBoardSingleton.getInstance(telemetry);
        this.limelight = new Limelight(hardwareMap,telemetry);
        this.intakeAndScoringSubsystem = new IntakeAndScoringSubsystem(hardwareMap,telemetry);
        robot = Robot.init(hardwareMap,telemetry, Robot.OpModeType.AUTO);
        this.mecanumDrive = new MecanumDrive(hardwareMap,telemetry);

        this.root = new Sequence(
                Arrays.asList(
                        new Selector(
                                Arrays.asList(
                                        new Conditional(new IsBotOriented()),
                                        new Action(new DetectYellowSamples(telemetry,limelight, Limelight.TargetPosition.SUBMERSIBLE),telemetry)),telemetry
                        ),
                        new Action(new RaiseArmToNeutralPosition(telemetry,intakeAndScoringSubsystem),telemetry),
                        new Action(new SquareBotWithSample(telemetry,mecanumDrive),telemetry),
                        new Action(new NavigateWithinRangeOfSample(telemetry,mecanumDrive),telemetry),
                        new Action(new SquareBotWithSample(telemetry,mecanumDrive),telemetry),
                        new Action(new OpenClaw(telemetry,intakeAndScoringSubsystem),telemetry),
                        new Action(new ExtendArmToSubmersibleSample(telemetry,intakeAndScoringSubsystem),telemetry),
                        new Action(new CloseClaw(telemetry,intakeAndScoringSubsystem),telemetry),
                        new Action(new RaiseArmToNeutralPosition(telemetry,intakeAndScoringSubsystem),telemetry)
                ),telemetry);

        this.tree = new BehaviorTree(root, blackBoard);
    }

    public Status tick() {
        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        for (LynxModule hub : robot.allHubs) {
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
