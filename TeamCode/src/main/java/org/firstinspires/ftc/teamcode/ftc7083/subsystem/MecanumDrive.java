package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionExBase;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDController;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDControllerImpl;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

import java.util.Arrays;
import java.util.Collection;

/**
 * MecanumDrive implements the drive chassis for the robot.
 */
@Config
public class MecanumDrive extends SubsystemBase {
    public static double POWER_EXPONENT = 1.0;
    public static double STRAFING_ADJUSTMENT = 1.1;
    public static double MAX_DRIVE_POWER_GAIN = 1.0;

    private final Telemetry telemetry;
    public final Motor rightFront, rightRear, leftFront, leftRear;
    private double driveGain = MAX_DRIVE_POWER_GAIN;

    /**
     * MecanumDrive initializes a new mecanum drive train.
     *
     * @param hardwareMap the hardware map that contains the drone launcher hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public MecanumDrive(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFront = new Motor(hardwareMap, telemetry, "leftFrontWheel");
        leftRear = new Motor(hardwareMap, telemetry, "leftRearWheel");
        rightFront = new Motor(hardwareMap, telemetry, "rightFrontWheel");
        rightRear = new Motor(hardwareMap, telemetry, "rightRearWheel");

        leftFront.setDirection(Motor.Direction.REVERSE);
        leftRear.setDirection(Motor.Direction.REVERSE);

        Collection<Motor> motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        for (Motor motor : motors) {
            initMotor(motor);
        }
    }

    /**
     * initMotor initializes a motor attached to the mecanum wheel.
     *
     * @param motor the motor to be initialized.
     */
    private void initMotor(@NonNull Motor motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * drive sets the powers to the wheel motors to result in the robot moving
     * in the direction provided on input.
     *
     * @param x    how much to move right or, if a negative value, left.
     * @param y    how much to move forward or, if a negative value, backward.
     * @param turn how much to rotate the robot.
     */
    public void drive(double x, double y, double turn) {
        x *= STRAFING_ADJUSTMENT; // Adjust for imperfect strafing

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = power * cos / max - turn;
        double rightFrontPower = power * sin / max + turn;
        double leftRearPower = power * sin / max - turn;
        double rightRearPower = power * cos / max + turn;

        // Normalize motor powers to ensure none exceeds 1.0
        double maxMotorPower = power + Math.abs(turn);
        if (maxMotorPower > 1) {
            leftFrontPower /= maxMotorPower;
            rightFrontPower /= maxMotorPower;
            leftRearPower /= maxMotorPower;
            rightRearPower /= maxMotorPower;
        }

        // Adjust the power applied to the motors. This raises the power to an exponent to create
        // an exponential curve, as well as applies the power gain to each motor's power.
        leftFrontPower = adjustMotorPower(leftFrontPower);
        rightFrontPower = adjustMotorPower(rightFrontPower);
        leftRearPower = adjustMotorPower(leftRearPower);
        rightRearPower = adjustMotorPower(rightRearPower);

        setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
    }

    /**
     * drive sets the powers to the wheel motors to result in the robot moving
     * in the direction provided on input.
     *
     * @param x    how much to move right or, if a negative value, left.
     * @param y    how much to move forward or, if a negative value, backward.
     * @param turn how much to rotate the robot.
     */
    public void driveWithoutAdjustment(double x, double y, double turn) {
        x *= STRAFING_ADJUSTMENT; // Adjust for imperfect strafing

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = power * cos / max - turn;
        double rightFrontPower = power * sin / max + turn;
        double leftRearPower = power * sin / max - turn;
        double rightRearPower = power * cos / max + turn;

        // Normalize motor powers to ensure none exceeds 1.0
        double maxMotorPower = power + Math.abs(turn);
        if (maxMotorPower > 1) {
            leftFrontPower /= maxMotorPower;
            rightFrontPower /= maxMotorPower;
            leftRearPower /= maxMotorPower;
            rightRearPower /= maxMotorPower;
        }

        setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
    }

    /**
     * Adjust the value to reduce sensitivity of the joystick when pushed only a small distance
     *
     * @param value the value based from the controller
     * @return the adjusted value
     */
    private double adjustMotorPower(double value) {
        double sign = value < 0 ? -1 : 1;
        double newPower = Math.pow(Math.abs(value), POWER_EXPONENT) * sign;
        newPower *= driveGain;
        return newPower;
    }

    /**
     * setMotorPowers sets the power for the wheels, normalizing for a maximum power of 1.0.
     *
     * @param leftFrontPower  the power for the left front motor.
     * @param leftRearPower   the power for the left rear motor
     * @param rightRearPower  the power for the right rear motor.
     * @param rightFrontPower the power for the right front motor.
     */
    public void setMotorPowers(double leftFrontPower, double leftRearPower, double rightRearPower, double rightFrontPower) {
        // Get the maximum power for any motor, or 1.0, whichever is greater
        double maxPower = maxAbs(1.0, leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);

        // Divide by the maximum power, which guarantees no motor's power will exceed 1.0.
        // This also ensures that all motors get a proportional amount of power should the
        // input power for any motor exceed 1.0.
        leftFrontPower /= maxPower;
        leftRearPower /= maxPower;
        rightFrontPower /= maxPower;
        rightRearPower /= maxPower;

        // Now that the power have been normalized, go ahead and set power for the motors.
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

        telemetry.addData("[DRIVE] Left Front Power", leftFrontPower);
        telemetry.addData("[DRIVE] Left Rear Power", leftRearPower);
        telemetry.addData("[DRIVE] Right Front Power", rightFrontPower);
        telemetry.addData("[DRIVE] Right Rear Power", rightRearPower);
    }

    /**
     * Turns the robot given the amount of power.
     *
     * @param turnPower the amount of power to apply to the wheel motors.
     */
    public void turn(double turnPower) {

        double leftFrontPower = turnPower;
        double rightFrontPower = turnPower;
        double leftRearPower = turnPower;
        double rightRearPower = turnPower;

        // Normalize motor powers to ensure none exceeds 1.0
        double maxMotorPower = Math.abs(turnPower);
        if (maxMotorPower > 1) {
            leftFrontPower /= maxMotorPower;
            rightFrontPower /= maxMotorPower;
            leftRearPower /= maxMotorPower;
            rightRearPower /= maxMotorPower;
        }

        setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
    }

    /**
     * Drives the robot in a straight direction.
     *
     * @param drivePower the amount of power to apply to the wheel motors.
     */
    public void driveStraight(double drivePower) {

        double leftFrontPower = drivePower;
        double rightFrontPower = drivePower;
        double leftRearPower = drivePower;
        double rightRearPower = drivePower;

        // Normalize motor powers to ensure none exceeds 1.0
        double maxMotorPower = Math.abs(drivePower);
        if (maxMotorPower > 1) {
            leftFrontPower /= maxMotorPower;
            rightFrontPower /= maxMotorPower;
            leftRearPower /= maxMotorPower;
            rightRearPower /= maxMotorPower;
        }

        setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);

    }

    /**
     * Gets the current power gain for the drive subsystem.
     *
     * @return the current power gain for the drive subsystem
     */
    public double getDriveGain() {
        return driveGain;
    }

    /**
     * Set the power gain for the drive subsystem. The power applied to the wheels is multiplied
     * by the gain to adjust the power that is set.
     *
     * @param gain the power gain to use
     * @return this mecanum drive instance
     */
    public MecanumDrive setDriveGain(double gain) {
        // Gain is required to be between [0,1]
        if (gain > MAX_DRIVE_POWER_GAIN) {
            gain = MAX_DRIVE_POWER_GAIN;
        } else if (gain < 0) {
            gain = 0;
        }
        this.driveGain = gain;
        return this;
    }

    /**
     * An action to drive the robot to a position where it can intake a sample.
     *
     * @param limelight the limelight camera used to detect the sample
     * @return an action to drive the robot to a position where it can intake a sample
     */
    public ActionEx actionDriveToSample(Limelight limelight) {
        return new DriveToSample(this, limelight, telemetry);
    }

    /**
     * An action to drive the robot to a position where it can intake a specimen.
     *
     * @param limelight the limelight camera used to detect the specimen
     * @return an action to drive the robot to a position where it can intake a specimen
     */
    public ActionEx actionDriveToSpecimen(Limelight limelight) {
        return new DriveToSpecimen(this, limelight, telemetry);
    }

    /**
     * Gets a string representation of this mecanum drive.
     *
     * @return a string representation of this mecanum drive
     */
    @NonNull
    @Override
    public String toString() {
        return "MecanumDrive{" +
                "rightFront=" + rightFront +
                ", rightRear=" + rightRear +
                ", leftFront=" + leftFront +
                ", leftRear=" + leftRear +
                '}';
    }

    /**
     * Drives the robot to a sample or specimen on the field
     */
    @Config
    public abstract static class DriveTo extends ActionExBase {
        // PID control values
        public static double KP = 0.0;
        public static double KI = 0.0;
        public static double KD = 0.0;
        public static double KF = 0.0;

        // Distance of the Limelight camera from the front of the robot
        public static double LIMELIGHT_DISTANCE_TO_FRONT_OF_ROBOT = 12.0;

        // Distances for driving
        public static double TOLERABLE_X_ERROR = 0.25; // inches
        public static double TOLERABLE_Y_ERROR = 0.25; // inches

        // PID controllers to determine power to the motors
        private final PIDController xController = new PIDControllerImpl(KP, KI, KD);
        private final PIDController yController = new PIDControllerImpl(KP, KI, KD);

        // Distances from the target to move the robot
        private final double xTargetDistance;
        private final double yTargetDistance;
        private final Limelight.TargetPosition targetPosition;

        private final MecanumDrive mecanumDrive;
        private final Limelight limelight;
        private final Telemetry telemetry;


        /**
         * Instantiates an action to drive the robot to a sample or specimen.
         *
         * @param mecanumDrive    the drive subsystem to use to move the robot
         * @param limelight       the Limelight camera used to detect the sample or specimen
         * @param telemetry       telemetry used to output data to the driver station
         * @param xTargetDistance distance from the target to which to move the robot along the X-axis
         * @param yTargetDistance distance from the target to which to move the robot along the Y-axis
         * @param targetPosition  the location of the target; samples are in the <code>SUBMERSIBLE</code>, while
         *                        specimens are <code>WALL</code>.
         */
        public DriveTo(MecanumDrive mecanumDrive, Limelight limelight, Telemetry telemetry,
                       double xTargetDistance, double yTargetDistance, Limelight.TargetPosition targetPosition) {
            this.mecanumDrive = mecanumDrive;
            this.limelight = limelight;
            this.telemetry = telemetry;
            this.xTargetDistance = xTargetDistance;
            this.yTargetDistance = yTargetDistance;
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Power to apply if no sample is detected
            double ZERO_POWER = KF; // TODO: set to 0, and make a constant

            // TODO: remove setting PID coefficients once tuned
            // Set the PID coefficients
            xController.setCoefficients(KP, KI, KD);
            yController.setCoefficients(KP, KI, KD);

            // Get the distance and angle from the sample
            Double distance = limelight.getDistance(targetPosition);
            Double angle = limelight.getTx();
            telemetry.addData("[Drive] Distance", distance);
            telemetry.addData("[Drive] Angle", angle);

            // Get the X and Y distances from the sample, if one is detected
            double x, y;
            if (distance != null && angle != null) {
                telemetry.addData("[Drive] sample detected", true);
                double theta = Math.toRadians(angle);
                x = distance * Math.cos(theta);
                y = distance * Math.sin(theta);
            } else {
                telemetry.addData("[Drive] sample detected", false);
                x = 0;
                y = 0;
            }

            // Drive to the location at which to pickup a sample
            double xPower;
            if (x > TOLERABLE_X_ERROR) {
                double pid = xController.calculate(x, 0.0);
                double ff = pid < 0 ? -KF : KF;
                xPower = pid + ff;
            } else {
                xPower = ZERO_POWER;
                xController.reset();
            }
            double yPower;
            if (y > yTargetDistance + TOLERABLE_Y_ERROR) {
                double pid = yController.calculate(x, 0.0);
                double ff = pid < 0 ? -KF : KF;
                yPower = pid + ff;
            } else {
                yPower = ZERO_POWER;
                yController.reset();
            }
            mecanumDrive.driveWithoutAdjustment(xPower, yPower, 0.0);

            telemetry.addData("[Drive] X-Target", xTargetDistance);
            telemetry.addData("[Drive] X-Current", x);
            telemetry.addData("[Drive] X-Power", xPower);
            telemetry.addData("[Drive] Y-Target", yTargetDistance);
            telemetry.addData("[Drive] Y-Current", y);
            telemetry.addData("[Drive] Y-Power", yPower);

            boolean atTarget = xPower == ZERO_POWER && yPower == ZERO_POWER;
            telemetry.addData("[Drive] atTarget", atTarget);

            return !atTarget;
        }
    }

    /**
     * Action to drive the robot to a sample found on the ground.
     */
    public static class DriveToSample extends DriveTo {
        // Distances for driving
        public static double TARGET_X_DISTANCE = 0.0; // inches, from the robot
        public static double TARGET_Y_DISTANCE = LIMELIGHT_DISTANCE_TO_FRONT_OF_ROBOT + 10.5; // inches, from the robot

        /**
         * Instantiates an action to drive to a sample on the ground.
         *
         * @param mecanumDrive the drive subsystem to use to move the robot
         * @param limelight    the Limelight camera used to detect the sample
         * @param telemetry    telemetry used to output data to the driver station
         */
        public DriveToSample(MecanumDrive mecanumDrive, Limelight limelight, Telemetry telemetry) {
            super(mecanumDrive, limelight, telemetry, TARGET_X_DISTANCE, TARGET_Y_DISTANCE, Limelight.TargetPosition.SUBMERSIBLE);
        }
    }

    /**
     * Action to drive the robot to a specimen found on the wall.
     */
    public static class DriveToSpecimen extends DriveTo {
        // Distances for driving
        public static double TARGET_X_DISTANCE = 0.0; // inches, from the robot
        public static double TARGET_Y_DISTANCE = LIMELIGHT_DISTANCE_TO_FRONT_OF_ROBOT + 4.0; // inches, from the robot

        /**
         * Instantiates an action to drive to a specimen on the wall.
         *
         * @param mecanumDrive the drive subsystem to use to move the robot
         * @param limelight    the Limelight camera used to detect the specimen
         * @param telemetry    telemetry used to output data to the driver station
         */
        public DriveToSpecimen(MecanumDrive mecanumDrive, Limelight limelight, Telemetry telemetry) {
            super(mecanumDrive, limelight, telemetry, TARGET_X_DISTANCE, TARGET_Y_DISTANCE, Limelight.TargetPosition.WALL);
        }
    }
}
