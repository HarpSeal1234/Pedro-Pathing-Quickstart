package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Camera Pivot Tuner", group = "OrcaRobotics")
public class CameraPivotTuner extends OpMode {
    private Servo pivot;
    private double pivotPos = 0.5;
    private static final double SMALL_STEP = 0.001;
    private static final double BIG_STEP = 0.01;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevUp = false;
    private boolean prevDown = false;

    @Override
    public void init(){
        pivot = hardwareMap.get(Servo.class, "pivot");
        pivot.setPosition(pivotPos);
        telemetry.addLine("Init Complete");
        telemetry.addLine("A/B: fine adjust (+/- 0.001)");
        telemetry.addLine("DPad Up/Down: coarse adjust (+/- 0.01)");
    }

    @Override
    public void loop(){
        // Fine adjust with A/B (edge detection)
        if (gamepad1.a && !prevA) {
            pivotPos += SMALL_STEP;
        }
        if (gamepad1.b && !prevB) {
            pivotPos -= SMALL_STEP;
        }
        // Coarse adjust with D-pad
        if (gamepad1.dpad_up && !prevUp) {
            pivotPos += BIG_STEP;
        }
        if (gamepad1.dpad_down && !prevDown) {
            pivotPos -= BIG_STEP;
        }

        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;

        // Clamp to valid servo range
        pivotPos = Math.max(0.0, Math.min(1.0, pivotPos));

        pivot.setPosition(pivotPos);

        telemetry.addData("Pivot Position", pivotPos);
        telemetry.addLine("A: +0.001 | B: -0.001");
        telemetry.addLine("DPad Up: +0.01 | DPad Down: -0.01");
        telemetry.update();
    }
}
