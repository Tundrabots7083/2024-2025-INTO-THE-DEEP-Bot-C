package org.firstinspires.ftc.teamcode.ftc7083.filter;

import androidx.annotation.NonNull;

/**
 * The Kalman filter is a recursive algorithm that estimates the state of a linear dynamic system
 * from a series of noisy measurements.
 */
public class KalmanFilter1 {
    // Initial values, used when resetting the Kalman filter
    private final double initialState; // Initial state estimate
    private final double initialErrorCovariance; // Initial error estimate
    // Covariance Estimates
    private final double processNoise;     // Process noise covariance
    private final double measurementNoise; // Measurement noise covariance
    // Current values
    private double state;            // Current state estimate
    private double errorCovariance;  // Current error

    /**
     * Creates a new Kalman filter
     *
     * @param initialState           the initial state estimate
     * @param initialErrorCovariance the initial error covariance
     * @param processNoise           the initial process noise covariance
     * @param measurementNoise       the initial measurement noise covariance
     */
    public KalmanFilter1(double initialState, double initialErrorCovariance, double processNoise, double measurementNoise) {
        this.initialState = initialState;
        this.initialErrorCovariance = initialErrorCovariance;

        this.state = initialState;
        this.errorCovariance = initialErrorCovariance;
        this.processNoise = processNoise;
        this.measurementNoise = measurementNoise;
    }

    /**
     * Gets the current state estimate.
     *
     * @return the current state estimate.
     */
    public double getState() {
        return state;
    }

    /**
     * Resets the Kalman filter to the initial state.
     */
    public void reset() {
        state = initialState;
        errorCovariance = initialErrorCovariance;
    }

    /**
     * Updates the current state based on the measurement.
     *
     * @param measurement the system measurement.
     */
    public void update(double measurement) {
        // Prediction
        double predictedState = state;
        double predictedErrorCovariance = errorCovariance + processNoise;

        // Update
        double kalmanGain = predictedErrorCovariance / (predictedErrorCovariance + measurementNoise);
        state = predictedState + kalmanGain * (measurement - predictedState);
        errorCovariance = (1 - kalmanGain) * predictedErrorCovariance;
    }

    /**
     * Gets a string representation of this kalman filter.
     *
     * @return a string representation of this kalman filter
     */
    @NonNull
    @Override
    public String toString() {
        return "KalmanFilter{" +
                "initialState=" + initialState +
                ", initialErrorCovariance=" + initialErrorCovariance +
                ", processNoise=" + processNoise +
                ", measurementNoise=" + measurementNoise +
                ", state=" + state +
                ", errorCovariance=" + errorCovariance +
                '}';
    }

    /*
    // Example usage
    public static void main(String[] args) {
        double initialState = 0.0;
        double initialErrorCovariance = 1.0;
        double processNoise = 0.01;
        double measurementNoise = 0.1;

        KalmanFilter kalmanFilter = new KalmanFilter(initialState, initialErrorCovariance, processNoise, measurementNoise);

        // Simulate measurements
        double[] measurements = {6.5, 6.1, 5.3, 5.4, 4.9, 4.1, 4.1, 3.8, 3.5, 3.4, 3.5};

        for (double measurement : measurements) {
            kalmanFilter.update(measurement);
            System.out.println("Filtered State: " + kalmanFilter.getState());
        }
    }
    */
}
