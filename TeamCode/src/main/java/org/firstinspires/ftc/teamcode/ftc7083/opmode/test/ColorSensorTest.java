package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name = "Color Sensor Test", group = "tests")
public class ColorSensorTest extends OpMode {
    public static float GAIN = 15.0f;
    public static float MIN_RED_COLOR_VALUE = 2.0f;
    public static float MIN_BLUE_COLOR_VALUE = 2.0f;
    public static float MIN_GREEN_COLOR_VALUE = 4.0f;
    public static float MIN_GREEN_SATURATION = 0.5f;
    public static float COLOR_MULTIPLIER = 100.0f;

    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private View relativeLayout;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Get the actual Color Sensor. Note that the REV V3 Color Sensor is also a Distance Sensor.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        distanceSensor = (DistanceSensor) colorSensor;

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Tell the sensor our desired gain value (normally you would do this during initialization,
        // not during the loop)
        colorSensor.setGain(GAIN);

        // convert the RGB values to HSV values.
        float[] hsvValues = {0F, 0F, 0F};
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Scale the colors to make the values easier to view and compareb
        float alpha = colors.alpha * COLOR_MULTIPLIER;
        float red = colors.red * COLOR_MULTIPLIER;
        float blue = colors.blue * COLOR_MULTIPLIER;
        float green = colors.green * COLOR_MULTIPLIER;

        // Send the info back to driver station using telemetry function.
        telemetry.addData("Clear", "%.3f", alpha);
        telemetry.addData("Red  ", "%.3f", red);
        telemetry.addData("Green", "%.3f", green);
        telemetry.addData("Blue ", "%.3f", blue);

        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", "%.3f", hsvValues[1]);
        telemetry.addData("Value", "%.3f", hsvValues[2]);

        telemetry.addData("Distance (cm)", "%.3f", distanceSensor.getDistance(DistanceUnit.CM));

        telemetry.addData("Sample Color", getColor());

        // Change the background color to match the color detected by the RGB sensor.
        // Pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(() -> relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, hsvValues)));

        telemetry.update();
    }

    @Override
    public void stop() {
        // Set the panel back to the default color
        relativeLayout.post(() -> relativeLayout.setBackgroundColor(Color.WHITE));
    }

    private SampleColor getColor() {
        // Scale the colors to a larger value for comparisons
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float red = colors.red * COLOR_MULTIPLIER;
        float blue = colors.blue * COLOR_MULTIPLIER;
        float green = colors.green * COLOR_MULTIPLIER;
        float maxColor = Math.max(Math.max(red, blue), green);

        // convert the RGB values to HSV values.
        float[] hsvValues = {0F, 0F, 0F};
        Color.colorToHSV(colors.toColor(), hsvValues);
        float saturation = hsvValues[1];

        // NOTE: YELLOW is a mixture of RED and GREEN, but with a larger component of GREEN.
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

    public enum SampleColor {
        NONE,
        BLUE,
        RED,
        YELLOW
    }
}
