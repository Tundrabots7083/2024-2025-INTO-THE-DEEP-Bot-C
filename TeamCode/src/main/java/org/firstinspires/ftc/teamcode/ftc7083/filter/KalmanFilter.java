package org.firstinspires.ftc.teamcode.ftc7083.filter;

import org.firstinspires.ftc.teamcode.ftc7083.utils.LinearRegression;
import org.firstinspires.ftc.teamcode.ftc7083.utils.SizedStack;

/**
 * This Kalman Filter implementation has three parameters to tune.
 * <ul>
 *     <li>
 *         Q is the sensor covariance, or how much we trust the sensor, low values for the sensor
 *         means that we believe the sensor will have lots of noise and vice versa.
 *     </li>
 *     <li>
 *         R is the model covariance or how much we trust the linear regression.
 *     </li>
 *     <li>
 *         N is the number of elements back we perform the regression on. In most cases, a value
 *         between <code>3</code> and <code>5</code> works best.
 *     </li>
 * </ul>
 */
public class KalmanFilter implements Filter {
    protected final double Q;
    protected final double R;
    protected final int N;
    protected final SizedStack<Double> estimates;
    protected double P = 1;
    protected double K = 0;
    protected double x;
    protected LinearRegression regression;

    /**
     * A kalman filter that uses a least squares regression as it's model.
     *
     * @param Q Sensor Covariance
     * @param R Model Covariance
     * @param N Number of elements we can hold in our stack.
     */
    public KalmanFilter(double Q, double R, int N) {
        this.Q = Q;
        this.R = R;
        this.N = N;
        this.x = 0;
        this.estimates = new SizedStack<>(N);
        initializeStackWith0();
        regression = new LinearRegression(stackToDoubleArray());
        findK();
    }

    /**
     * initialize the stack to all 0's
     */
    protected void initializeStackWith0() {
        for (int i = 0; i < N; ++i) {
            estimates.push(0.0);
        }
    }

    /**
     * convert the stack to an array of doubles
     *
     * @return an array of doubles.
     */
    protected double[] stackToDoubleArray() {
        double[] newValues = new double[N];
        for (int i = 0; i < estimates.size(); ++i) {
            newValues[i] = estimates.get(i);
        }
        return newValues;
    }

    /**
     * Iteratively compute K using the D.A.R.E
     */
    public void findK() {
        for (int i = 0; i < 2000; ++i) solveDARE();
    }

    /**
     * solve the discrete time algebraic riccati equation (D.A.R.E)
     */
    public void solveDARE() {
        P = P + Q;
        K = P / (P + R);
        P = (1 - K) * P;
    }

    /**
     * get the state estimate.
     *
     * @return state estimate
     */
    public double getX() {
        return x;
    }

    /**
     * set the state estimate.
     *
     * @param x state estimate
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * update the kalman filter for traditional; continuous values.
     *
     * @param measurement the current measurement
     * @return the optimal state estimate
     */
    @Override
    public double estimate(double measurement) {
        regression.runLeastSquares();
        x += regression.predictNextValue() - estimates.peek();
        x += K * (measurement - x);
        estimates.push(x);
        regression = new LinearRegression(stackToDoubleArray());
        return x;
    }
}
