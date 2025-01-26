package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTree.WristOrientationBehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTree.WristOrientationBehaviorTreeSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Status;

@Autonomous(name = "Wrist Orientation Subsystem BT", group = "Test")
public class WristOrientationTestSubsystem extends LinearOpMode {

    WristOrientationBehaviorTreeSubsystem behaviorTree = null;

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
        this.behaviorTree = new WristOrientationBehaviorTreeSubsystem(hardwareMap,telemetry);
    }
}
