package org.firstinspires.ftc.teamcode.ftc7083.filter;

/**
 * Interface for filters
 */
public interface Filter {
    /**
     * Estimate the filter value given the measurement
     *
     * @param measurement the measurement to filter
     * @return the filtered value for the measurement
     */
    double estimate(double measurement);
}
