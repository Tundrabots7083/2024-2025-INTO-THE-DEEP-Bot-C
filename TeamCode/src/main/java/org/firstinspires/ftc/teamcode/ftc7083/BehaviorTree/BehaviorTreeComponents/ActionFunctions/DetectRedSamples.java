package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Limelight;

public class DetectRedSamples implements ActionFunction {
    private Limelight limelight;
    private Telemetry telemetry;
    private Limelight.TargetPosition targetHeight;
    private int count=0;
    private Double xDistance;
    private Double Tx;

    public DetectRedSamples(Telemetry telemetry, Limelight limelight, Limelight.TargetPosition targetHeight) {
        this.limelight = limelight;
        this.telemetry = telemetry;
        this.targetHeight = targetHeight;
    }

    public Status perform(BlackBoardSingleton blackBoard) {

        telemetry.addData("[DetectRedSamples] "," perform count: %d", count);
        telemetry.update();
        count++;

        limelight.detectRed();

        xDistance = limelight.getDistance(targetHeight);
        Tx = limelight.getTx();

        if( Tx != null && xDistance != null){

            blackBoard.setValue("xDistanceToSample", xDistance);
            blackBoard.setValue("Tx",Tx);

            telemetry.addData("[DetectSamples]","X-Distance to Sample: %f",xDistance);
            telemetry.addData("[DetectRedSamples]","Tx-Angle from Sample: %f",Tx);
            telemetry.update();

            return Status.SUCCESS;
        } else {
            blackBoard.setValue("xDistanceToSample", null);
            blackBoard.setValue("Tx",null);

            telemetry.addData("[DetectYellowSamples]","Didn't detect anything");
            telemetry.update();

            if(count >= 20) {
                return Status.FAILURE;
            } else {
                return Status.RUNNING;
            }
        }
    }


}
