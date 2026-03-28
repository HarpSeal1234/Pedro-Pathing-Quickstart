package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TurretScaleCalculator", group="! Linear OpMode")
public class TurretScaleCalculator extends LinearOpMode {
    private Servo turretServo = null;
    private double servoPosition = 0.5; // Initial position (center)

    @Override
    public void runOpMode() throws InterruptedException {
        turretServo = hardwareMap.get(Servo.class,"turretServo");
        turretServo.setPosition(servoPosition); // Center position
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a){
                servoPosition += 0.001; // Increment position
                if (servoPosition > 1.0) {
                    servoPosition = 1.0; // Cap at max
                }
                turretServo.setPosition(servoPosition);
            } else if(gamepad1.b){
                servoPosition -= 0.001; // Decrement position
                if (servoPosition < 0.0) {
                    servoPosition = 0.0; // Cap at min
                }
                turretServo.setPosition(servoPosition);
            }
            telemetry();

            telemetry.update();
        }
    }
    public void telemetry() {
        telemetry.addData("Turret Position", turretServo.getPosition());
    }
}
