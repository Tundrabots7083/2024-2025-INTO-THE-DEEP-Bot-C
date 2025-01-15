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
    public static double PITCH_DEGREES_OFFSET = 150.0;
    public static double PITCH_MAX_DEGREES = 355;
    public static double PITCH_SERVO_DEGREES = 0.0;
    public static double ROLL_DEGREES_OFFSET = 156.0;
    public static double ROLL_MAX_DEGREES = 355;
    public static double ROLL_SERVO_DEGREES = 0.0;

    private Servo rollServo;
    private Servo pitchServo;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pitchServo = new Servo(hardwareMap, WRIST_PITCH_SERVO_NAME);
        pitchServo.setMaxDegrees(PITCH_MAX_DEGREES);
        pitchServo.setDirection(Servo.Direction.REVERSE);
        rollServo = new Servo(hardwareMap, WRIST_ROLL_SERVO_NAME);
        rollServo.setMaxDegrees(ROLL_MAX_DEGREES);
        rollServo.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        rollServo.setMaxDegrees(ROLL_MAX_DEGREES);
        pitchServo.setMaxDegrees(PITCH_MAX_DEGREES);

        pitchServo.setDegrees(PITCH_SERVO_DEGREES + PITCH_DEGREES_OFFSET);
        rollServo.setDegrees(ROLL_SERVO_DEGREES + ROLL_DEGREES_OFFSET);

        telemetry.addData("Target pitchDegrees", PITCH_SERVO_DEGREES);
        telemetry.addData("Target rollDegrees", ROLL_SERVO_DEGREES);
        telemetry.addData("Retrieved pitchDegrees", pitchServo.getDegrees() - PITCH_DEGREES_OFFSET);
        telemetry.addData("Retrieved rollDegrees", rollServo.getDegrees() - ROLL_DEGREES_OFFSET);
        telemetry.addData("Retrieved rollPosition", rollServo.getPosition());
        telemetry.addData("Retrieved pitchPosition", pitchServo.getPosition());
        telemetry.update();
    }
}
