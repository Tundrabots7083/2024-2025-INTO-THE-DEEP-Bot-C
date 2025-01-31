package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.GlobalShutterCamera;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.processors.SampleProcessor;

public class DetectSampleOrientation implements ActionFunction {
    GlobalShutterCamera globalShutterCamera;
    private Telemetry telemetry;
    private int count=0;
    private AllianceColor allianceColor;
    protected Status lastStatus = Status.FAILURE;

    public DetectSampleOrientation(Telemetry telemetry, GlobalShutterCamera globalShutterCamera,AllianceColor allianceColor) {
        this.globalShutterCamera = globalShutterCamera;
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;

    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status;

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        telemetry.addData("[DetectSampleOrientation]", " perform count: %d", count);
        telemetry.update();
        count++;

        SampleProcessor sampleProcessor = new SampleProcessor();
         Double yellowAngle = sampleProcessor.execute(globalShutterCamera.getYellowDetections());
         telemetry.addData("Yellow Angle", yellowAngle);

         if(allianceColor == AllianceColor.BLUE) {
             Double blueAngle = sampleProcessor.execute(globalShutterCamera.getBlueDetections());
             telemetry.addData("Blue Angle", blueAngle);
         } else {
             Double redAngle = sampleProcessor.execute(globalShutterCamera.getRedDetections());
             telemetry.addData("Red Angle", redAngle);
         }

        if (yellowAngle != null) {
            yellowAngle -= 90;
            blackBoard.setValue("SampleAngle", yellowAngle);
            status = Status.SUCCESS;

        } else {
            blackBoard.setValue("SampleAngle", null);

            telemetry.addData("[DetectSampleOrientation]", "Didn't detect anything");
            telemetry.update();

            if (count >= 50) {
                status = Status.FAILURE;
            } else {
                status = Status.RUNNING;
            }
        }

        lastStatus = status;
        return status;
    }

}

