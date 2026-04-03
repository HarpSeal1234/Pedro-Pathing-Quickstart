package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.CONSTANTS.BLUE_GOAL_POSITION_X;
import static org.firstinspires.ftc.teamcode.CONSTANTS.BLUE_GOAL_POSITION_Y;
import static org.firstinspires.ftc.teamcode.CONSTANTS.HOOD_MAX_POS;
import static org.firstinspires.ftc.teamcode.CONSTANTS.HOOD_MIN_POS;
import static org.firstinspires.ftc.teamcode.CONSTANTS.MAX_TURRET_ANGLE;
import static org.firstinspires.ftc.teamcode.CONSTANTS.TURRET_POSITION_PER_DEGREE;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Launcher {
    public enum LauncherState {
        START_LAUNCHING_NEAR,
        START_LAUNCHING_FAR,
        LAUNCH,
        LAUNCH_BLUE_FAR,
        IDLE,
        PICKUP
    }

    private LauncherState launcherState = LauncherState.IDLE;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;
    private DcMotor intake1;
    private DcMotor intake2;
    private Servo turretServo = null;
    private Servo hoodServo = null;
    private Timer stateTimer = new Timer();
    private Timer launchTimer = new Timer();
    public final static double JR_OUTTAKE_BLOCK = 0.78;
    public final static double JR_OUTTAKE_OPEN = 0.0;

    public Launcher(HardwareMap hardwareMap) {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setDirection(DcMotorEx.Direction.FORWARD);
        outtake1.setVelocity(0);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setVelocity(0);

        intake1 = hardwareMap.get(DcMotorEx.class,"intake1");
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2 = hardwareMap.get(DcMotor.class,"intake2");
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretServo = hardwareMap.get(Servo.class,"turretServo");

        hoodServo = hardwareMap.get(Servo.class,"hoodServo");
    }

    public void setState(LauncherState state) {
        launcherState = state;
        stateTimer.resetTimer();
    }

    public void update(){
        switch (launcherState){
            case START_LAUNCHING_NEAR:
                outtake1.setVelocity(1600);
                outtake2.setVelocity(1600);
                intake1.setPower(0);
                intake2.setPower(0);
                turretServo.setPosition(Range.clip(0.42, 0.28, 0.694)); // 0.422
                hoodServo.setPosition(Range.clip(0.62,HOOD_MIN_POS,HOOD_MAX_POS));
                break;
            case START_LAUNCHING_FAR:
                outtake1.setVelocity(2200);
                outtake2.setVelocity(2200);
                intake1.setPower(0);
                intake2.setPower(0);
                turretServo.setPosition(Range.clip(0.4, 0.28, 0.694)); // 0.422
                hoodServo.setPosition(Range.clip(0.4,HOOD_MIN_POS,HOOD_MAX_POS));
                break;
            case LAUNCH:
                intake1.setPower(1);
                intake2.setPower(1);
                break;
            case IDLE:
                intake1.setPower(0);
                intake2.setPower(0);
                break;
            case PICKUP:
//                blocker.setPosition(JR_OUTTAKE_BLOCK);
                intake1.setPower(1);
                intake2.setPower(0);
        }
    }
    public LauncherState getState() {
        return launcherState;
    }

    public void updateTurret(Pose robotPose){
         double turretPos = 0.5;
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading();
        double angleToGoal = Math.atan2(robotX - BLUE_GOAL_POSITION_X, BLUE_GOAL_POSITION_Y - robotY) + Math.PI / 2;

        double turretError = angleToGoal - robotHeading;
        while (turretError > Math.PI) turretError -= 2 * Math.PI;
        while (turretError < -Math.PI) turretError += 2 * Math.PI;

        double errorDegrees = Math.toDegrees(turretError);

// Clamp turret angle to ±135° to prevent over-rotation
        errorDegrees = Range.clip(errorDegrees, -MAX_TURRET_ANGLE, MAX_TURRET_ANGLE);

        turretPos = 0.5 + (errorDegrees * TURRET_POSITION_PER_DEGREE);
        turretServo.setPosition(Range.clip(turretPos, 0.28, 0.694));
    }

}
