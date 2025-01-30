package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDControllerImpl;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;

@Config
public class NavigateWithinRangeOfSample implements ActionFunction {

    MecanumDrive mecanumDrive;
    Telemetry telemetry;

    public static double KP = 0.01;
    public static double KI = 0;
    public static double KD = 0;
    public static double TOLERABLE_ERROR = 0.5; // inches
    public static double MAXIMUM_INTAKE_DISTANCE = 25; //inches

    private final PIDControllerImpl pidController;

    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;

    public NavigateWithinRangeOfSample(Telemetry telemetry, MecanumDrive mecanumDrive) {
        this.mecanumDrive = mecanumDrive;
        this.telemetry = telemetry;
        pidController = new PIDControllerImpl(KP, KI, KD);
        pidController.reset();
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status = Status.RUNNING;
        double xDistanceToSample;
        double distanceError;

        if (lastStatus == Status.SUCCESS) {
            telemetry.addData("[NavigateWithinRange] status:", "Just left it as success");
            telemetry.update();
            return lastStatus;
        }


        if (blackBoard.getValue("xDistanceToSample") != null) {
            xDistanceToSample = (double) blackBoard.getValue("xDistanceToSample");
            distanceError = xDistanceToSample - MAXIMUM_INTAKE_DISTANCE;
            telemetry.addData("[Navigate]distance",xDistanceToSample);
            telemetry.addData("[Navigate]distanceError",distanceError);
            telemetry.update();
        } else {
            status = Status.FAILURE;
            blackBoard.setValue("BotIsInIntakeRange",false);
            return status;
        }

        if (distanceError <= TOLERABLE_ERROR) {
            mecanumDrive.driveStraight(0.0);
            status = Status.SUCCESS;
            blackBoard.setValue("BotIsInIntakeRange",true);
            lastStatus = status;
            return status;
        }

        double drivePower = pidController.calculate(MAXIMUM_INTAKE_DISTANCE, distanceError);

        mecanumDrive.driveStraight(drivePower);

        runCount++;

        telemetry.addData("[NavigateWithinRange] status:", status);
        telemetry.update();
        lastStatus = status;

        return status;
    }
}
