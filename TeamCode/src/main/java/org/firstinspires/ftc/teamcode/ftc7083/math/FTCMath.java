package org.firstinspires.ftc.teamcode.ftc7083.math;

import java.math.BigDecimal;
import java.math.RoundingMode;

/**
 * Math routines used in FTC by Tundrabots
 */
public class FTCMath {
    /**
     * Round a number to the specified number of decimal places.
     *
     * @param value  the value to round
     * @param places the number of decimal places to which to round the number
     * @return the rounded number
     */
    public static double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = BigDecimal.valueOf(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}
