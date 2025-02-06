package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTree.IntakeSampleBehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;

import java.util.List;

@Autonomous (name = "Intake Sample BT", group = "tests")
public class IntakeSampleBehaviorTreeTest extends LinearOpMode {

    IntakeSampleBehaviorTree behaviorTree = null;

    int loopCount = 0;

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        initialize();

        waitForStart();

        while (opModeIsActive()) {

            // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
            // as the bulk read caches are being handled manually.
        List<LynxModule> allHubs;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

            Status result = this.behaviorTree.tick();

            telemetry.addData("[IntakeSampleBT]", "Behavior tree result: %s", result);

            telemetry.addData("[IntakeSampleBT]", "loop count: %d", this.loopCount);
            telemetry.update();

            loopCount++;

            if (result == Status.SUCCESS || result == Status.FAILURE) {
                telemetry.addData("[IntakeSampleBT]", "runOpMode success");
                telemetry.update();
                requestOpModeStop();

            }
        }
    }

    private void initialize(){

        Robot.init(hardwareMap,telemetry);

        this.behaviorTree = new IntakeSampleBehaviorTree(hardwareMap,telemetry);

        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        List<LynxModule> allHubs;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

    }

}
