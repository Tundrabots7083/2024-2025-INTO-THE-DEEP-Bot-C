package org.firstinspires.ftc.teamcode.ftc7083.feedback;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * A PID controller to calculate the output given the target state and current state.
 */
public class PIDControllerImpl implements PIDController {
    private static final double MIN_INTEGRAL_SUM = -1.0;
    private static final double MAX_INTEGRAL_SUM = 1.0;

    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double Kp;
    private double Ki;
    private double Kd;
    private boolean hasRun = false;
    private double previousError = 0;

    private double integralSum = 0;

    /**
     * Creates a new PID controller.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change
     */
    public PIDControllerImpl(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    @Override
    public double calculate(double reference, double state) {
        if (Double.isNaN(reference)) {
            return 0;
        }

        double dt = getDT();
        double error = calculateError(reference, state);
        double derivative = calculateDerivative(error, dt);
        integrate(error, dt);
        previousError = error;

        return error * Kp
                + integralSum * Ki
                + derivative * Kd;
    }

    /**
     * Gets the time constant
     *
     * @return time constant
     */
    private double getDT() {
        if (!hasRun) {
            hasRun = true;
            timer.reset();
        }
        double dt = timer.time();
        timer.reset();
        return dt;
    }

    @Override
    public void reset() {
        hasRun = false;
        timer.reset();
    }

    @Override
    public void setCoefficients(double Kp, double Ki, double Kd) {
        if (this.Kp != Kp || this.Ki != Ki || this.Kd != Kd) {
            this.Kp = Kp;
            this.Kd = Kd;
            this.Ki = Ki;
//            reset();
        }
    }

    /**
     * Calculates the error between the target and the current states.
     *
     * @param reference the target state.
     * @param state     the current state.
     * @return the difference between the target state and the current state.
     */
    protected double calculateError(double reference, double state) {
        return reference - state;
    }

    /**
     * Adds the current error to the integral sum.
     *
     * @param error the current error
     * @param dt    the time since the PID controller last updated.
     */
    private void integrate(double error, double dt) {
        integralSum += ((error + previousError) / 2) * dt;
        integralSum = Range.clip(integralSum, MIN_INTEGRAL_SUM, MAX_INTEGRAL_SUM);
    }

    /**
     * Calculates the derivative.
     *
     * @param error the current error
     * @param dt    the time since the PID controller last updated.
     * @return the derivative.
     */
    private double calculateDerivative(double error, double dt) {
        double derivative = (error - previousError) / dt;

        if (Double.isNaN(derivative)) {
            derivative = 0;
        }

        return derivative;
    }

    /**
     * Gets a string representation of this PID controller.
     *
     * @return a string representation of this PID controller
     */
    @NonNull
    @Override
    public String toString() {
        return "PIDController{" +
                "Kp=" + Kp +
                ", Kd=" + Kd +
                ", Ki=" + Ki +
                '}';
    }
}
