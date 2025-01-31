package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTree.WristOrientationBehaviorTreeSamples;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;

@Autonomous(name = "Wrist Orientation BT", group = "Test")
public class WristOrientationTest extends LinearOpMode {

    WristOrientationBehaviorTreeSamples behaviorTree = null;

    int loopCount = 0;

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        initialize();

        telemetry.addLine("Robot Initialized");

        waitForStart();

        while (opModeIsActive()) {

            Status result = this.behaviorTree.tick();

            telemetry.addData("[Wrist Orientation BT]", "Behavior tree result: %s", result);

            telemetry.addData("[Wrist Orientation BT]", "loop count: %d", this.loopCount);
            telemetry.update();

            loopCount++;

            if (result == Status.SUCCESS || result == Status.FAILURE) {
                telemetry.addData("[Wrist Orientation BT]", "runOpMode success");
                telemetry.update();
                requestOpModeStop();
            }
        }
    }

    private void initialize(){
        this.behaviorTree = new WristOrientationBehaviorTreeSamples(hardwareMap,telemetry);
    }
}
