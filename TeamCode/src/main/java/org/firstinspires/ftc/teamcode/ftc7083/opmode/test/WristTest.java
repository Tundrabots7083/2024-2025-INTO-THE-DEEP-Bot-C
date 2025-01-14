package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.hardware.Servo;

@Config
@TeleOp(name = "Wrist Test", group = "tests")
public class WristTest extends OpMode {
    public static String WRIST_PITCH_SERVO_NAME = "wristPitch";
    public static String WRIST_ROLL_SERVO_NAME = "wristRoll";
    // public static double SERVO_MAX_DEGREES = 270;
    public static double PITCH_MAX_DEGREES = 270;
    public static double ROLL_MAX_DEGREES = 355;
    public static double PITCH_SERVO_DEGREES = 0.0;
    public static double ROLL_SERVO_DEGREES = 0.0;

    private Servo rollServo;
    private Servo pitchServo;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rollServo = new Servo(hardwareMap, WRIST_ROLL_SERVO_NAME);
        pitchServo = new Servo(hardwareMap, WRIST_PITCH_SERVO_NAME);
        rollServo.setMaxDegrees(ROLL_MAX_DEGREES);
        pitchServo.setMaxDegrees(PITCH_MAX_DEGREES);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        rollServo.setMaxDegrees(ROLL_MAX_DEGREES);
        pitchServo.setMaxDegrees(PITCH_MAX_DEGREES);
        pitchServo.setDegrees(PITCH_SERVO_DEGREES);
        rollServo.setDegrees(ROLL_SERVO_DEGREES);
        telemetry.addData("Target Degrees", PITCH_SERVO_DEGREES);
        telemetry.addData("Retrieved pitchDegrees", pitchServo.getDegrees());
        telemetry.addData("Retrieved rollDegrees", rollServo.getDegrees());
        telemetry.addData("rollPosition", rollServo.getPosition());
        telemetry.addData("pitchPosition", pitchServo.getPosition());
        telemetry.update();
    }
}
