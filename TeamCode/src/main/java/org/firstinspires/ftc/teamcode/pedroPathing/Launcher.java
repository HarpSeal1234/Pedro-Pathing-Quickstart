package org.firstinspires.ftc.teamcode.pedroPathing;

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
        IDLE,
        PICKUP
    }

    private LauncherState launcherState = LauncherState.IDLE;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;
    private DcMotor intake1;
    private DcMotor intake2;
//    private Servo blocker = null;
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
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2 = hardwareMap.get(DcMotor.class,"intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        blocker = hardwareMap.get(Servo.class,"blocker");
//        blocker.setPosition(Range.clip(JR_OUTTAKE_BLOCK,0,1));
    }

    public void setState(LauncherState state) {
        launcherState = state;
        stateTimer.resetTimer();
    }

    public void update(){
        switch (launcherState){
            case START_LAUNCHING_NEAR:
                outtake1.setVelocity(1100);
                outtake2.setVelocity(1100);
                intake1.setPower(0);
                intake2.setPower(0);
//                if (outtake1.getVelocity() > 100){
//                    setState(LauncherState.LAUNCH);
//                }

                break;
            case START_LAUNCHING_FAR:
                outtake1.setVelocity(1400);
                outtake2.setVelocity(1400);
                intake1.setPower(0);
                intake2.setPower(0);
                if (stateTimer.getElapsedTime() > 200){
                    setState(LauncherState.LAUNCH);
                }
            case LAUNCH:
                intake1.setPower(1);
                intake2.setPower(1);
//                blocker.setPosition(JR_OUTTAKE_OPEN);
//                if (stateTimer.getElapsedTime() > 100){
//                    setState(LauncherState.IDLE);
//                }
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
}
