package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDControllerImpl;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;

@Config
public class NavigateAndOrientToSpecimen implements ActionFunction {

    MecanumDrive mecanumDrive;
    Telemetry telemetry;

    public static double KPDrive = 0.019;
    public static double KIDrive = 0.09;
    public static double KDDrive = 0.0;
    public static double KPStrafe = 0.017;
    public static double KIStrafe = 0.09;
    public static double KDStrafe = 0.0;
    public static double TOLERABLE_ERROR = 0.5; // inches
    public static double MAXIMUM_INTAKE_DISTANCE = 30; //inches
    public static double INTAKE_ANGLE = 0.0;

    private final PIDControllerImpl DrivePIDController;
    private final PIDControllerImpl TurnPIDController;

    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;

    public NavigateAndOrientToSpecimen(Telemetry telemetry, MecanumDrive mecanumDrive) {
        this.mecanumDrive = mecanumDrive;
        this.telemetry = telemetry;
        DrivePIDController = new PIDControllerImpl(KPDrive, KIDrive, KDDrive);
        DrivePIDController.reset();

        TurnPIDController = new PIDControllerImpl(KPStrafe,KIStrafe,KDStrafe);
        TurnPIDController.reset();
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status = Status.RUNNING;
        Double xDistanceToSample;
        double distanceError;
        double strafeError;
        double drivePower = 0.0;
        double strafePower = 0.0;

        if (lastStatus == Status.SUCCESS) {
            telemetry.addData("[NavigateWithinRange] status:", "Just left it as success");
            telemetry.update();
            return lastStatus;
        }



            xDistanceToSample = (Double) blackBoard.getValue("xDistanceToSample");
            if (xDistanceToSample > 67) {
                xDistanceToSample = null;
            }
        if (xDistanceToSample != null) {
            distanceError = xDistanceToSample - MAXIMUM_INTAKE_DISTANCE;
            telemetry.addData("[Navigate]distance",xDistanceToSample);
            telemetry.addData("[Navigate]distanceError",distanceError);
            telemetry.update();
        } else {
            status = Status.FAILURE;
            blackBoard.setValue("BotIsInIntakeRange",false);
            return status;
        }

        if (blackBoard.getValue("Tx") != null) {
            strafeError = (double) blackBoard.getValue("Tx");
        } else {
            status = Status.FAILURE;
            blackBoard.setValue("BotIsSquaredWithSample",false);
            return status;
        }

        if (distanceError <= TOLERABLE_ERROR && Math.abs(strafeError) <= TOLERABLE_ERROR) {
            mecanumDrive.driveStraight(0.0);
            status = Status.SUCCESS;
            blackBoard.setValue("BotIsInIntakeRange",true);
            blackBoard.setValue("BotIsSquaredWithSample", true);
            lastStatus = status;
            return status;
        }

        if (distanceError >= TOLERABLE_ERROR) {
            drivePower = DrivePIDController.calculate(0, -distanceError);
        }
        if (Math.abs(strafeError) >= TOLERABLE_ERROR) {
            strafePower = TurnPIDController.calculate(INTAKE_ANGLE, strafeError);
        }

        mecanumDrive.driveWithoutAdjustment(strafePower,drivePower, 0);

        runCount++;

        telemetry.addData("[NavigateWithinRange] status:", status);
        telemetry.update();
        lastStatus = status;

        return status;
    }
}
