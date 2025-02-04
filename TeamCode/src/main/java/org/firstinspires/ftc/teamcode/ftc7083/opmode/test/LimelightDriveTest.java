package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDController;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDControllerImpl;
import org.firstinspires.ftc.teamcode.ftc7083.math.FTCMath;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Limelight;

/**
 * A test OpMode that drives the robot to a detected sample on the field. The color of the
 * sample to drive to is set based on <code>Robot.INTAKE_COLOR</code>.
 */
@Config
@TeleOp(name = "Limelight Drive", group = "tests")
public class LimelightDriveTest extends OpMode {
    // PID control values
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static double LIMELIGHT_DISTANCE_TO_FRONT_OF_ROBOT = 12.0;

    // Distances for driving
    public static double TARGET_X_DISTANCE = 0.0; // inches, from the robot
    public static double TARGET_Y_DISTANCE = LIMELIGHT_DISTANCE_TO_FRONT_OF_ROBOT + 10.5; // inches, from the robot
    public static double TOLERABLE_X_ERROR = 0.25; // inches
    public static double TOLERABLE_Y_ERROR = 0.25; // inches

    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final PIDController xController = new PIDControllerImpl(kP, kI, kD);
    private final PIDController yController = new PIDControllerImpl(kP, kI, kD);
    private Robot robot;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot = Robot.init(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        switch (Robot.INTAKE_COLOR) {
            case RED:
                robot.limelight.detectRed();
                break;
            case BLUE:
                robot.limelight.detectBlue();
                break;
            default:
                robot.limelight.detectYellow();
        }
    }

    @Override
    public void loop() {
        timer.reset();

        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        for (LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }

        xController.setCoefficients(kP, kI, kD);
        yController.setCoefficients(kP, kI, kD);

        // Get the distance and angle from the sample
        Double hypotenuse = robot.limelight.getDistance(Limelight.TargetPosition.SUBMERSIBLE);
        Double angle = robot.limelight.getTx();
        telemetry.addData("Distance", hypotenuse);
        telemetry.addData("Angle", angle);

        // If a sample is detected, drive to it
        if (hypotenuse != null && angle != null) {
            double theta = Math.toRadians(angle);
            double x = hypotenuse * Math.cos(theta);
            double y = hypotenuse * Math.sin(theta);
            driveToSample(x, y);
        } else {
            telemetry.addData("Drive", "no sample");
        }

        // Print out the loop time, in milliseconds, to two decimal places
        telemetry.addData("Loop Time", FTCMath.round(timer.time(), 2));
        telemetry.update();
    }

    /**
     * Drive to the sample.
     *
     * @param x the distance along the X-axis to the sample
     * @param y the distance along the Y-axis to the sample
     */
    public void driveToSample(double x, double y) {
        // Drive to the location at which to pickup a sample
        double xPower;
        if (x > TOLERABLE_X_ERROR) {
            xPower = xController.calculate(x, 0.0) + kF;
        } else {
            xPower = 0.0;
            xController.reset();
        }
        double yPower;
        if (y > TARGET_Y_DISTANCE + TOLERABLE_Y_ERROR) {
            yPower = yController.calculate(y, TARGET_Y_DISTANCE) + kF;
        } else {
            yPower = 0.0;
            yController.reset();
        }
        robot.mecanumDrive.driveWithoutAdjustment(xPower, yPower, 0.0);

        if (xPower == 0.0 && yPower == 0.0) {
            telemetry.addData("Drive", "at target");
        } else {
            telemetry.addData("Drive", "move to target");
        }

        telemetry.addData("xTarget", TARGET_X_DISTANCE);
        telemetry.addData("xCurrent", x);
        telemetry.addData("xPower", xPower);
        telemetry.addData("yTarget", TARGET_Y_DISTANCE);
        telemetry.addData("yCurrent", y);
        telemetry.addData("yPower", yPower);
    }
}
