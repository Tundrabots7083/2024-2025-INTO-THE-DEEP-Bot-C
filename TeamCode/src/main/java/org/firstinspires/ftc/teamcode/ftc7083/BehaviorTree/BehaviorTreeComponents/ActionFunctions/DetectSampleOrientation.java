package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.GlobalShutterCamera;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.processors.SampleProcessor;

public class DetectSampleOrientation implements ActionFunction {
    GlobalShutterCamera globalShutterCamera;
    private Telemetry telemetry;
    private int count;
    protected Status lastStatus = Status.FAILURE;
    private SampleProcessor sampleProcessor = new SampleProcessor();;

    public DetectSampleOrientation(Telemetry telemetry, GlobalShutterCamera globalShutterCamera) {
        this.globalShutterCamera = globalShutterCamera;
        this.telemetry = telemetry;
        this.count = 0;

    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status;

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        telemetry.addData("[DetectSampleOrientation]", " perform count: %d", count);
        telemetry.update();
        count++;

        Double blueAngle = null;
        Double redAngle = null;

         Double yellowAngle = sampleProcessor.execute(globalShutterCamera.getYellowDetections());
         telemetry.addData("Yellow Angle", yellowAngle);

         switch (Robot.INTAKE_COLOR) {
             case BLUE:
                 blueAngle = sampleProcessor.execute(globalShutterCamera.getBlueDetections());
                 telemetry.addData("Blue Angle", blueAngle);
                 break;
             case RED:
                 redAngle = sampleProcessor.execute(globalShutterCamera.getRedDetections());
                 telemetry.addData("Red Angle", redAngle);
                 break;
             case YELLOW:
                 blueAngle = sampleProcessor.execute(globalShutterCamera.getBlueDetections());
                 telemetry.addData("Blue Angle", blueAngle);
                 redAngle = sampleProcessor.execute(globalShutterCamera.getRedDetections());
                 telemetry.addData("Red Angle", redAngle);
                 break;
             default:
                 telemetry.addLine("[DetectSampleOrientation] No Sample Intake Color Set");
         }

         telemetry.update();

        if (yellowAngle != null) {
            yellowAngle -= 90;
            blackBoard.setValue("SampleAngle", yellowAngle);
            lastStatus = Status.SUCCESS;
            return lastStatus;
        } else if (blueAngle != null) {
            blueAngle -= 90;
            blackBoard.setValue("SampleAngle", blueAngle);
            lastStatus = Status.SUCCESS;
            return lastStatus;
        } else if (redAngle != null) {
            redAngle -= 90;
            blackBoard.setValue("SampleAngle", redAngle);
            lastStatus = Status.SUCCESS;
            return lastStatus;
        }  else {
            blackBoard.setValue("SampleAngle", null);

            telemetry.addData("[DetectSampleOrientation]", "Didn't detect anything");
            telemetry.update();

            if (count >= 75) {
                status = Status.FAILURE;
            } else {
                status = Status.RUNNING;
            }
        }

        lastStatus = status;
        return status;
    }

}

