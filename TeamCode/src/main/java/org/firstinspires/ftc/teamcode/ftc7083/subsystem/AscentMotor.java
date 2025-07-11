package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.FeedForward;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.profile.MotionProfile;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * A linear slide can extend and retract the wrist and claw attached to the robot's scoring
 * subsystem.
 */
@Config
public class AscentMotor extends SubsystemBase {

    private final Motor ascentMotor;
    private final Telemetry telemetry;
    private Gamepad gamepad;

    double power = 0;


    /**
     * Instantiates the linear slide for the robot with the specified feed forward.
     *
     * @param hardwareMap  Hardware Map
     * @param telemetry    Telemetry
     */
    public AscentMotor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        ascentMotor = new Motor(hardwareMap, telemetry, "ascentMotor");
        configMotor(ascentMotor);
    }

    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void setPower(double power) {
        ascentMotor.setPower(power);
    }

    /**
     * Configures the motor used for the linear slide
     *
     * @param motor the motor to be configured
     */
    private void configMotor(Motor motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motor.setMotorType(motorConfigurationType);
        motor.setMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(Motor.Direction.REVERSE);
    }

    /**
     * sets the power for the pid controller
     */
    @Override
    public void execute() {
        if(gamepad != null) {
            setPower(gamepad.right_trigger);
        }
         telemetry.addData("[Ascent Motor] Power", power);
        }
    }
