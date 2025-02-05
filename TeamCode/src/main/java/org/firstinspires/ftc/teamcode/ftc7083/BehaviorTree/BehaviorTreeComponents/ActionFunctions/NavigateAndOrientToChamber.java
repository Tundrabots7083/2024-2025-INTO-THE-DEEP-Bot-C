package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDControllerImpl;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.profile.OLD_PIDFController;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;

import java.util.Objects;

@Config
public class NavigateAndOrientToChamber implements ActionFunction {

    MecanumDrive mecanumDrive;
    Telemetry telemetry;

    public static double KPDrive = 0.019;
    public static double KIDrive = 0.09;
    public static double KDDrive = 0.0;
    public static double KPTurn = 0.019;
    public static double KITurn = 0.09;
    public static double KDTurn = 0.0;
    private final PIDCoefficients DRIVEpidCoefficients = new PIDCoefficients(KPDrive,KIDrive,KDDrive);
    private final PIDCoefficients TURNpidCoefficients = new PIDCoefficients(KPTurn,KITurn,KDTurn);
    public static double TOLERABLE_ERROR = 0.5; // inches
    public static double MAXIMUM_DISTANCE_TO_CHAMBER = 30; //inches
    private double INTAKE_ANGLE = 0.0;

    private final OLD_PIDFController DRIVEoldPidfController;
    private final OLD_PIDFController TURNoldPidfController;



    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;

    public NavigateAndOrientToChamber(Telemetry telemetry, MecanumDrive mecanumDrive) {
        this.mecanumDrive = mecanumDrive;
        this.telemetry = telemetry;

        DRIVEoldPidfController = new OLD_PIDFController(DRIVEpidCoefficients,0,0,0.1);
        DRIVEoldPidfController.reset();

        TURNoldPidfController = new OLD_PIDFController(TURNpidCoefficients,0,0,0.1);
        TURNoldPidfController.reset();
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status = Status.RUNNING;
        Double currentXPose;
        Double currentHeading;
        double distanceError;
        double turnError;
        double drivePower;
        double turnPower;


        if (lastStatus == Status.SUCCESS) {
            telemetry.addData("[NavigateWithinRange] status:", "Just left it as success");
            telemetry.update();
            return lastStatus;
        }



            currentXPose = (Double) blackBoard.getValue("Current_X_pose");
            currentHeading = (Double) blackBoard.getValue("Current_Heading");

            if (currentXPose > 67) {
                currentXPose = null;
            }
        if (currentXPose != null) {
            distanceError = currentXPose - MAXIMUM_DISTANCE_TO_CHAMBER;
            turnError = currentHeading - INTAKE_ANGLE;
            telemetry.addData("[Navigate] Current_X_pose",currentXPose);
            telemetry.addData("[Navigate] distanceError",distanceError);
            telemetry.update();
        } else {
            status = Status.FAILURE;
            blackBoard.setValue("BotIsInIntakeRange",false);
            return status;
        }


        if (distanceError <= TOLERABLE_ERROR && Math.abs(turnError) <= TOLERABLE_ERROR) {
            mecanumDrive.driveStraight(0.0);
            status = Status.SUCCESS;
            blackBoard.setValue("BotIsInIntakeRange",true);
            blackBoard.setValue("BotIsSquaredWithSample", true);
            lastStatus = status;
            return status;
        }

        if (distanceError >= TOLERABLE_ERROR) {
            drivePower = DRIVEoldPidfController.update(distanceError);
        } else {
            drivePower = 0.0;
        }
        if (Math.abs(turnError) >= TOLERABLE_ERROR) {
            turnPower = TURNoldPidfController.update(turnError);
        } else {
            turnPower = 0.0;
        }

        mecanumDrive.driveWithoutAdjustment(0,drivePower, turnPower);

        runCount++;

        telemetry.addData("[NavigateWithinRange] status:", status);
        telemetry.update();
        lastStatus = status;

        return status;
    }
}
