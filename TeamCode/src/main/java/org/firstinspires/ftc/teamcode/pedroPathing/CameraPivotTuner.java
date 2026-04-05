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
    private static final double step = 0.001;

    @Override
    public void init(){
        pivot = hardwareMap.get(Servo.class, "pivot");
        pivot.setPosition(pivotPos); // set to middle position
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop(){
        // get gamepad commands
        if (gamepad1.aWasPressed()){
            pivotPos += step;
        }else if(gamepad1.bWasPressed()){
            pivotPos -= step;
        }

        pivot.setPosition(pivotPos);


        telemetry.addData("Pivot Position", pivotPos);
        telemetry.update();
    }
}
