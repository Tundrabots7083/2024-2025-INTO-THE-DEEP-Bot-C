package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.ColorSensor;

@Config
@TeleOp(name = "Color Sensor Test", group = "tests")
public class ColorSensorTest extends OpMode {
    private ColorSensor colorSensor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Get the actual Color Sensor. Note that the REV V3 Color Sensor is also a Distance Sensor.
        colorSensor = new ColorSensor(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Tell the sensor our desired gain value (normally this is done within the ColorSensor
        // class itself)
        colorSensor.setGain(ColorSensor.DEFAULT_GAIN);

        sendDistanceSensorTelemetry();
        sendColorSensorTelemetry();

        telemetry.update();
    }

    /**
     * Sends telemetry data related to the color sensor
     */
    public void sendColorSensorTelemetry() {
        // Convert the RGB values to HSV values
        float[] hsvValues = {0F, 0F, 0F};
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Scale the colors to make the values easier to view.
        float alpha = colors.alpha * ColorSensor.COLOR_MULTIPLIER;
        float red = colors.red * ColorSensor.COLOR_MULTIPLIER;
        float blue = colors.blue * ColorSensor.COLOR_MULTIPLIER;
        float green = colors.green * ColorSensor.COLOR_MULTIPLIER;

        // Send the info back to driver station using telemetry function.
        telemetry.addData("Clear", "%.3f", alpha);
        telemetry.addData("Red  ", "%.3f", red);
        telemetry.addData("Green", "%.3f", green);
        telemetry.addData("Blue ", "%.3f", blue);

        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", "%.3f", hsvValues[1]);
        telemetry.addData("Value", "%.3f", hsvValues[2]);

        telemetry.addData("Sample Color", colorSensor.getColor());

    }

    /**
     * Sends telemetry data related to the distance sensor
     */
    public void sendDistanceSensorTelemetry() {
        telemetry.addData("Distance (cm)", "%.3f", colorSensor.getDistance(DistanceUnit.CM));
    }
}
