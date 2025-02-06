package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.ArmWithProfile;

@Config
@TeleOp(name = "Arm Tuning", group = "tests")
public class ArmTuning extends OpMode {
    public static double ACHIEVABLE_MAX_RPM_FRACTION = 1.0;
    public static double TICKS_PER_REV = 537.7; // GoBuilda ticks per rev
    public static double KG = 0.17;

    public static double GEARING = 22.0 / 10.0;

    public static double ARM_ANGLE = ArmWithProfile.START_ANGLE;
    private Motor motor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = new Motor(hardwareMap, telemetry, "arm");


        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setTicksPerRev(TICKS_PER_REV);
        motorConfigurationType.setGearing(GEARING);
        motorConfigurationType.setAchieveableMaxRPMFraction(ACHIEVABLE_MAX_RPM_FRACTION);
        motor.setMotorType(motorConfigurationType);
        motor.setMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(Motor.Direction.FORWARD);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        motor.setPower(KG);
    }

}
