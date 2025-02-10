package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.feedback.LinearSlideFeedForward;

@Config
@TeleOp(name = "Tune Linear Slide PID", group="tuning")
public class TuneLinearSlidePID extends OpMode {
    // Arm PID values
    public static double ARM_KP = 0;
    public static double ARM_KI = 0;
    public static double ARM_KD = 0;
    public static double ARM_KS = 0;
    public static double ARM_KG = Arm.KG;

    // Linear Slide PID values
    public static double LINEAR_SLIDE_KP = 0;
    public static double LINEAR_SLIDE_KI = 0;
    public static double LINEAR_SLIDE_KD = 0;
    public static double LINEAR_SLIDE_KS = 0;
    public static double LINEAR_SLIDE_KG = 0;

    // Angle to which to move the arm
    public static double LINEAR_SLIDE_LENGTH = 0;

    private Arm arm;
    private LinearSlide linearSlide;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Arm.KP = ARM_KP;
        Arm.KI = ARM_KI;
        Arm.KD = ARM_KD;
        Arm.KS = ARM_KS;
        Arm.KG = ARM_KG;
        arm = new Arm(hardwareMap, telemetry);

        LinearSlide.KP = LINEAR_SLIDE_KP;
        LinearSlide.KI = LINEAR_SLIDE_KI;
        LinearSlide.KD = LINEAR_SLIDE_KD;
        LinearSlide.KS = LINEAR_SLIDE_KS;
        LinearSlide.KG = LINEAR_SLIDE_KG;
        linearSlide = new LinearSlide(hardwareMap, telemetry, new LinearSlideFeedForward(arm, LinearSlide.KG));

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        Arm.KP = ARM_KP;
        Arm.KI = ARM_KI;
        Arm.KD = ARM_KD;
        Arm.KS = ARM_KS;
        Arm.KG = ARM_KG;
        arm.resetPID();

        LinearSlide.KP = LINEAR_SLIDE_KP;
        LinearSlide.KI = LINEAR_SLIDE_KI;
        LinearSlide.KD = LINEAR_SLIDE_KD;
        LinearSlide.KS = LINEAR_SLIDE_KS;
        LinearSlide.KG = LINEAR_SLIDE_KG;
        linearSlide.resetPID(new LinearSlideFeedForward(arm, LinearSlide.KG));

        linearSlide.setLength(LINEAR_SLIDE_LENGTH);

        telemetry.addData("Target", linearSlide.getTargetLength());
        telemetry.addData("Current", linearSlide.getCurrentLength());

        telemetry.update();
    }
}
