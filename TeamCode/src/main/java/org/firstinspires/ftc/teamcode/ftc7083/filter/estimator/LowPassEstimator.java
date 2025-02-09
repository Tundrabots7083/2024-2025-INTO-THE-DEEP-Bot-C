package org.firstinspires.ftc.teamcode.ftc7083.filter.estimator;

import org.firstinspires.ftc.teamcode.ftc7083.filter.LowPassFilter;

import java.util.function.DoubleSupplier;

/**
 * Filters high frequency noise.
 */
public class LowPassEstimator extends Estimator {

    protected LowPassFilter filter;

    /**
     * Set up Double Supplier for recurring measurement obtainment.
     * <p>
     * Uses a low pass filter to estimate the systems state.
     *
     * @param measurement measurement we want to obtain.
     */
    public LowPassEstimator(DoubleSupplier measurement, double LowPassGain) {
        super(measurement);
        filter = new LowPassFilter(LowPassGain);
    }

    @Override
    public double update() {
        return filter.estimate(measurement.getAsDouble());
    }
}
