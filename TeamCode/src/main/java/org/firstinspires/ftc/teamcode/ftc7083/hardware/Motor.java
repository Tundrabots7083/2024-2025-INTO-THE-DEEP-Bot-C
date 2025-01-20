package org.firstinspires.ftc.teamcode.ftc7083.hardware;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Motor implements DcMotorEx {
    private final DcMotorEx motorImpl;
    private final Telemetry telemetry;
    private double inchesPerRev;

    /**
     * Instantiate a new motor for the robot.
     *
     * @param hardwareMap the mapping for all hardware on the robot
     * @param telemetry   the telemetry used to output data to the user
     * @param deviceName  the name of the motor as configured via the Driver Station
     */
    public Motor(HardwareMap hardwareMap, Telemetry telemetry, String deviceName) {
        this(hardwareMap.get(DcMotorEx.class, deviceName), telemetry);
    }

    /**
     * Instantiate a new motor for the robot.
     *
     * @param motorImpl the motor as retrieved via the hardware map
     */
    protected Motor(DcMotorEx motorImpl, Telemetry telemetry) {
        this.motorImpl = motorImpl;
        this.telemetry = telemetry;
    }

    /**
     * Gets the number of motor ticks required to move what is attached to the motor one inch.
     *
     * @return the number of motor ticks required to move what is attached to the motor one inch
     */
    public double getInchesPerRev() {
        return inchesPerRev;
    }

    /**
     * Sets the number of inches that the hardware attached to the motor moves for every rotation
     * of the motor.
     *
     * @param inchesPerRev the number of inches moved per revolution of the motor
     */
    public void setInchesPerRev(double inchesPerRev) {
        this.inchesPerRev = inchesPerRev;
    }

    /**
     * Gets the number of motor ticks required to move what is attached to the motor one degree
     * of rotation.
     *
     * @return number of motor ticks to move one degree of rotation
     */
    public double getDegreesPerRev() {
        MotorConfigurationType config = motorImpl.getMotorType();
        double gearing = config.getGearing(); // 5.0
        return 360.0 / gearing;
    }

    @Override
    public void setMotorEnable() {
        motorImpl.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        motorImpl.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return motorImpl.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        motorImpl.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return motorImpl.getVelocity();
    }

    @Override
    public void setVelocity(double angularRate) {
        motorImpl.setVelocity(angularRate);
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return motorImpl.getVelocity(unit);
    }

    @Override
    @Deprecated
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        motorImpl.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        motorImpl.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        motorImpl.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        motorImpl.setPositionPIDFCoefficients(p);
    }

    @Override
    @Deprecated
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return motorImpl.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return motorImpl.getPIDFCoefficients(mode);
    }

    @Override
    public int getTargetPositionTolerance() {
        return motorImpl.getTargetPositionTolerance();
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        motorImpl.setTargetPosition(tolerance);
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return motorImpl.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motorImpl.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        motorImpl.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return motorImpl.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motorImpl.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motorImpl.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return motorImpl.getController();
    }

    @Override
    public int getPortNumber() {
        return motorImpl.getPortNumber();
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motorImpl.getZeroPowerBehavior();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        motorImpl.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    @Deprecated
    public void setPowerFloat() {
        motorImpl.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return motorImpl.getPowerFloat();
    }

    /**
     * Gets the current degree offset of the motor.
     * <p>
     * Before using, make sure you call <code>setTicksPerDegree</code> to set the number of ticks
     * required to move whatever is attached to the motor one degree.
     *
     * @return the current degree offset of the motor
     */
    public double getCurrentDegrees() {
        MotorConfigurationType motorType = motorImpl.getMotorType();
        double ticksPerRev = motorType.getTicksPerRev();
        double gearing = motorType.getGearing();

        double currentTicks = getCurrentPosition();
        double rotations = currentTicks / ticksPerRev;
        double degreesPerRotation = 360.0 / gearing;
        return rotations * degreesPerRotation;
    }

    /**
     * Gets the target position of the motor in degrees.
     * <p>
     * Before using, make sure you call <code>setTicksPerDegree</code> to set the number of ticks
     * required to move whatever is attached to the motor one degree.
     *
     * @return the target degree offset of the motor
     */
    public double getTargetDegrees() {
        MotorConfigurationType motorType = motorImpl.getMotorType();
        double ticksPerRev = motorType.getTicksPerRev();
        double gearing = motorType.getGearing();

        double targetTicks = motorImpl.getTargetPosition();
        double rotations = targetTicks / ticksPerRev;
        double degreesPerRotation = 360.0 / gearing;
        return rotations * degreesPerRotation;
    }

    /**
     * Sets the position of the motor to the specified number of degrees.
     * <p>
     * Before using, make sure you call <code>setTicksPerDegree</code> to set the number of ticks
     * required to move whatever is attached to the motor one degree.
     *
     * @param degrees the degrees to which to set the motor
     */
    public void setTargetDegrees(double degrees) {
        MotorConfigurationType motorType = motorImpl.getMotorType();
        double ticksPerRev = motorType.getTicksPerRev();
        double gearing = motorType.getGearing();

        double outputTicksPerRev = ticksPerRev * gearing;
        double rotations = degrees / 360.0;
        double outputTicks = outputTicksPerRev * rotations;
        setTargetPosition((int) outputTicks);
    }

    /**
     * Gets the current inches the motor has moved from the zero position.
     *
     * <p>
     * Before using, make sure you call <code>setTicksPerInches</code> to set the number of ticks
     * required to move whatever is attached to the motor one inch.
     *
     * @return the current inches the motor has moved from the zero position
     */
    public double getCurrentInches() {
        double ticks = getCurrentPosition();
        double ticksPerRev = motorImpl.getMotorType().getTicksPerRev();
        double rotations = ticks / ticksPerRev;
        return rotations * inchesPerRev;
    }

    /**
     * Gets the target inches the motor is to move from the zero position.
     *
     * <p>
     * Before using, make sure you call <code>setTicksPerInches</code> to set the number of ticks
     * required to move whatever is attached to the motor one inch.
     *
     * @return the target inches the motor is to move from the zero position
     */
    public double getTargetInches() {
        double ticks = getTargetPosition();
        double ticksPerRev = motorImpl.getMotorType().getTicksPerRev();
        double rotations = ticks / ticksPerRev;
        return rotations * inchesPerRev;
    }

    /**
     * Sets the position of the motor to the specified number of inches.
     * <p>
     * Before using, make sure you call <code>setTicksPerInches</code> to set the number of ticks
     * required to move whatever is attached to the motor one inch.
     *
     * @param inches the inches to which to set the motor
     */
    public void setTargetInches(double inches) {
        double rotations = inches / inchesPerRev;
        double ticksPerRev = motorImpl.getMotorType().getTicksPerRev();
        double ticks = rotations * ticksPerRev;
        setTargetPosition((int) ticks);
    }

    @Override
    public int getTargetPosition() {
        return motorImpl.getTargetPosition();
    }

    @Override
    public void setTargetPosition(int position) {
        motorImpl.setTargetPosition(position);
    }

    @Override
    public boolean isBusy() {
        return motorImpl.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return motorImpl.getCurrentPosition();
    }

    @Override
    public RunMode getMode() {
        return motorImpl.getMode();
    }

    @Override
    public void setMode(RunMode mode) {
        motorImpl.setMode(mode);
    }

    @Override
    public Direction getDirection() {
        return motorImpl.getDirection();
    }

    @Override
    public void setDirection(Direction direction) {
        motorImpl.setDirection(direction);
    }

    @Override
    public double getPower() {
        return motorImpl.getPower();
    }

    @Override
    public void setPower(double power) {
        motorImpl.setPower(power);
    }

    @Override
    public Manufacturer getManufacturer() {
        return motorImpl.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motorImpl.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motorImpl.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motorImpl.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motorImpl.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        motorImpl.close();
    }
}
