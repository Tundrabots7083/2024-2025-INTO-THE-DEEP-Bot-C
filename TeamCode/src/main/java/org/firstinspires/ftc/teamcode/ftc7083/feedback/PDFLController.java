package org.firstinspires.ftc.teamcode.ftc7083.feedback;

import com.acmerobotics.dashboard.config.Config;

/**
 * A PID controller that includes a static component and feed forward.
 * <p>
 * The static component addresses friction on the system. This will vary based on the direction
 * of movement, where the friction varies based on the direction the mechanism is moving.
 * <p>
 * The feed forward component is intended to address the effects of gravity on the system, and
 * can vary depending on the type of mechanism being used. For example, a slide will typically have
 * a constant feed forward component, whereas an arm may change as the arm rotates through its
 * motion (e.g., the gravity affecting the arm is highest when horizontal, and zero when vertical).
 */
@Config
public class PDFLController extends PIDControllerEx {
    public static double DEFAULT_DEADZONE = 1e-6;
    private double kS;
    private double deadzone = DEFAULT_DEADZONE;

    /**
     * Instantiates a PDFL controller that does not use the integral and differential components.
     *
     * @param kP proportional term, multiplied directly by the state error
     * @param kS static term, with the opposite sign added based on the direction of movement
     * @param kF feed forward term, added to the output of the PID calculation
     */
    public PDFLController(double kP, double kS, double kF) {
        this(kP, 0, 0, kF, kS);
    }

    /**
     * Instantiates a PDFL controller that does not use the integral and differential components.
     *
     * @param kP proportional term, multiplied directly by the state error
     * @param kI integral term, multiplied directly by the state error integral
     * @param kD derivative term, multiplied directly by the state error rate of change
     * @param kS static term, with the opposite sign added based on the direction of movement
     * @param kF feed forward term, added to the output of the PID calculation
     */
    public PDFLController(double kP, double kI, double kD, double kS, double kF) {
        this(kP, kI, kD, kS, p -> kF);
    }

    /**
     * Creates a new PID controller with the provided FeedForward function.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change
     * @param kS static term, with the opposite sign added based on the direction of movement
     * @param ff the feed forward function
     */
    public PDFLController(double Kp, double Ki, double Kd, double kS, FeedForward ff) {
        super(Kp, Ki, Kd, ff);
        this.kS = kS;
    }

    /**
     * Sets a deadzone for the static component in the PDFL controller.
     *
     * @param deadzone the deadzone for the static component in the PDFL controller.
     */
    public void setDeadzone(double deadzone) {
        this.deadzone = deadzone;
    }

    @Override
    public double calculate(double reference, double state) {
        double pidf = super.calculate(reference, state);
        return pidf + staticComponent(pidf);
    }

    /**
     * Sets the PID coefficients to the new values.
     *
     * @param kP proportional term, multiplied directly by the state error
     * @param kI integral term, multiplied directly by the state error integral
     * @param kD derivative term, multiplied directly by the state error rate of change
     * @param kS static term, with the opposite sign added based on the direction of movement
     * @param kF feed forward term, added to the output of the PID calculation
     */
    public void setCoefficients(double kP, double kI, double kD, double kS, double kF) {
        setCoefficients(kP, kI, kD, kS, p-> kF);
    }

    /**
     * Sets the PID coefficients to the new values.
     *
     * @param kP proportional term, multiplied directly by the state error
     * @param kI integral term, multiplied directly by the state error integral
     * @param kD derivative term, multiplied directly by the state error rate of change
     * @param kS static term, with the opposite sign added based on the direction of movement
     * @param kF feed forward term, added to the output of the PID calculation
     */
    public void setCoefficients(double kP, double kI, double kD, double kS, FeedForward kF) {
        setCoefficients(kP, kI, kD, kF);
        this.kS = kS;
    }

    /**
     * Gets the static component based on the PIDF calculation.
     *
     * @param pidf the value calculated by the PIDF controller
     * @return the static component based on the PIDF calculation
     */
    private double staticComponent(double pidf) {
        if (Math.abs(pidf) >= deadzone) {
            return Math.copySign(kS, pidf);
        } else {
            return 0.0;
        }
    }
}
