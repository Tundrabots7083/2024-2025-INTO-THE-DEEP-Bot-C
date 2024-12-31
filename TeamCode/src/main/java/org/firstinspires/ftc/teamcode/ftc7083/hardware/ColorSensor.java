package org.firstinspires.ftc.teamcode.ftc7083.hardware;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * A color sensor that also serves as a distance sensor.
 */
@Config
public class ColorSensor implements NormalizedColorSensor, DistanceSensor {
    public static final String COLOR_SENSOR_NAME = "colorSensor";

    public static float DEFAULT_GAIN = 15.0f;
    public static float MIN_RED_COLOR_VALUE = 2.0f;
    public static float MIN_BLUE_COLOR_VALUE = 2.0f;
    public static float MIN_GREEN_COLOR_VALUE = 4.0f;
    public static float MIN_GREEN_SATURATION = 0.5f;
    public static float COLOR_MULTIPLIER = 100.0f;

    private final NormalizedColorSensor colorSensorImpl;

    public ColorSensor(HardwareMap hardwareMap, Telemetry telemetry) {
        colorSensorImpl = hardwareMap.get(NormalizedColorSensor.class, COLOR_SENSOR_NAME);
        colorSensorImpl.setGain(DEFAULT_GAIN);
    }

    @Override
    public NormalizedRGBA getNormalizedColors() {
        return colorSensorImpl.getNormalizedColors();
    }

    @Override
    public float getGain() {
        return colorSensorImpl.getGain();
    }

    @Override
    public void setGain(float newGain) {
        colorSensorImpl.setGain(newGain);
    }

    @Override
    public Manufacturer getManufacturer() {
        return colorSensorImpl.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return colorSensorImpl.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return colorSensorImpl.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return colorSensorImpl.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        colorSensorImpl.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        colorSensorImpl.close();
    }

    /**
     * Returns the current distance in centimeters.
     *
     * @return the current distance sas measured by the sensor. If no reading is available
     *         (perhaps the sensor is out of range), then distanceOutOfRange is returned;
     */
    public double getDistance() {
        return getDistance(DistanceUnit.CM);
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        DistanceSensor distanceSensorImpl = (DistanceSensor) colorSensorImpl;
        return distanceSensorImpl.getDistance(unit);
    }

    /**
     * Returns indicator as to whether a red sample is detected.
     *
     * @return <code>true</code> if a red sample is detected; <code>false</code> otherwise
     */
    public boolean isRed() {
        return getColor() == SampleColor.RED;
    }

    /**
     * Returns indicator as to whether a yellow sample is detected.
     *
     * @return <code>true</code> if a yellow sample is detected; <code>false</code> otherwise
     */
    public boolean isYellow() {
        return getColor() == SampleColor.YELLOW;
    }

    /**
     * Returns indicator as to whether a blue sample is detected.
     *
     * @return <code>true</code> if a blue sample is detected; <code>false</code> otherwise
     */
    public boolean isBlue() {
        return getColor() == SampleColor.BLUE;
    }

    /**
     * Returns the color of the sample that is detected. <code>SampleColor.NONE</code> is returned
     * if no sample is detected.
     *
     * @return the color of the sample that is detected, or <code>SampleColor.NONE</code> if no
     *         sample is detected
     */
    public SampleColor getColor() {
        // Scale the colors to a larger value for comparisons
        NormalizedRGBA colors = colorSensorImpl.getNormalizedColors();
        float red = colors.red * COLOR_MULTIPLIER;
        float blue = colors.blue * COLOR_MULTIPLIER;
        float green = colors.green * COLOR_MULTIPLIER;
        float maxColor = Math.max(Math.max(red, blue), green);

        // convert the RGB values to HSV values
        float[] hsvValues = {0F, 0F, 0F};
        Color.colorToHSV(colors.toColor(), hsvValues);
        float saturation = hsvValues[1];

        // NOTE: YELLOW is a mixture of RED and GREEN, but with a larger component of GREEN. To
        // not detect the tile color as YELLOW, a minimum saturation of green must be present as well.
        if (maxColor == red && red >= MIN_RED_COLOR_VALUE) {
            return SampleColor.RED;
        } else if (maxColor == green && green >= MIN_GREEN_COLOR_VALUE && saturation >= MIN_GREEN_SATURATION) {
            return SampleColor.YELLOW;
        } else if (maxColor == blue && blue >= MIN_BLUE_COLOR_VALUE) {
            return SampleColor.BLUE;
        } else {
            return SampleColor.NONE;
        }
    }

    /**
     * Indicator as to the color of the sample that is detected. If no sample is detected, then
     * <code>NONE</code> is used.
     */
    public enum SampleColor {
        NONE,
        BLUE,
        RED,
        YELLOW
    }
}
