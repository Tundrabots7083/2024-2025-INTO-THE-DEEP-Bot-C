package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.localization.Localizer;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.GlobalShutterCamera;

public class DetectAprilTags_GlobalShutter implements ActionFunction {
    private GlobalShutterCamera globalShutterCamera;
    private Robot robot;
    private Telemetry telemetry;
    private int count;
    protected Status lastStatus = Status.FAILURE;
    private Localizer localizer;
    private Pose2d currentPose;

    public DetectAprilTags_GlobalShutter(Telemetry telemetry, Robot robot) {
        this.globalShutterCamera = robot.aprilTagGlobalShutterCamera;
        this.robot = robot;
        localizer = robot.localizer;
        this.telemetry = telemetry;
        this.count = 0;

    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status;

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        telemetry.addData("[DetectAprilTags]", " perform count: %d", count);
        telemetry.update();
        count++;

        currentPose = localizer.getPose();

        blackBoard.setValue("Current_X_pose",currentPose.position.x);
        blackBoard.setValue("Current_Y_pose",currentPose.position.y);
        blackBoard.setValue("Current_Heading",currentPose.heading);



         telemetry.update();

        blackBoard.setValue("Current_X_pose",currentPose.position.x);

            telemetry.addData("[DetectAprilTags]", "Didn't detect anything");
            telemetry.update();

            if (count >= 75) {
                status = Status.FAILURE;
            } else {
                status = Status.RUNNING;
            }

        lastStatus = status;
        return status;
    }

}

