/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.CONSTANTS.CLOSE_INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.CONSTANTS.CLOSE_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.CONSTANTS.DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.CONSTANTS.FAR_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.CONSTANTS.JR_OUTTAKE_OPEN;
import static org.firstinspires.ftc.teamcode.CONSTANTS.kD;
import static org.firstinspires.ftc.teamcode.CONSTANTS.kI;
import static org.firstinspires.ftc.teamcode.CONSTANTS.kP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Adjustment Tele", group="! Linear OpMode")
public class AdjustmentTele extends LinearOpMode {
    private Follower follower;
    private double headingOffset = 0;
    private double turnP = 1.2;
    public static Pose startingPose;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront = null;

    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private Servo blocker = null;

    //    private Limelight3A limelight3A = null;
    private DcMotor leftBack = null;
    private DcMotorEx outtake1 = null;
    private DcMotorEx outtake2 = null;
    private DcMotorEx intake1 = null;
    private DcMotor intake2 = null;
    double intake1Power = 0.0;
    double intake2Power = 0.0;
    double targetv = 0;
    double lastTargetV = 0;
    boolean autoUpdate = false;
    ElapsedTime velocityTimer = new ElapsedTime();
    enum INTAKE_STATUS {
        INTAKE_STOPPED,
        INTAKE_STARTED,
        INTAKE_NEED_TO_BE_MONITORED,
        INTAKE_MONITORING,
        INTAKE_JAMMED
    }
    double targetOuttakeVelocity = 0.0;

    // pidf
    private double outtakeZeroPower = 0.0;
    private double motorOneCurrentVelocity = 0.0;
    private double motorOneMaxVelocity = 2800;
    private double F = 32767/motorOneMaxVelocity;

    private double position = 5.0;
    private double blockerPosition = JR_OUTTAKE_OPEN;

    boolean intake1On = false;
    double intake1Vel = 0.0;
    INTAKE_STATUS intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
    private Servo pivot;

    ElapsedTime intakeTimer = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        follower = Constants.createFollower(hardwareMap);
        startingPose = new Pose(72,72,Math.toRadians(90));
        double shootHeading = 138.5;
        boolean aiming = false;
        follower.setStartingPose(startingPose); // Or your last Auto pose
//        startingPose = follower.getPose();
        PathChain shoot;
        initHardware();
        follower.startTeleopDrive();



        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            follower.update();
            //drive
            double y = gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            intake1Vel = intake1.getVelocity();
//            targetv = Range.clip((((1450.0 - 1100) / (130 - 42)) * (getRobotToGoalDistance() - 42) + 1200), 1000, FAR_OUTTAKE_VELOCITY);

//            leftFront.setPower(leftFrontPower);
//            leftBack.setPower(leftBackPower);
//            rightFront.setPower(rightFrontPower);
//            rightBack.setPower(rightBackPower);


            if (gamepad1.right_trigger > 0.3) {
                aiming = true;
            } else {
                aiming = false;
            }
            if (aiming) {
                double targetX = 0;
                double targetY = 144;
                Pose pose = follower.getPose();

                // 1. Calculate the angle we WANT to be at
                double targetAngle = Math.atan2(targetY - pose.getY(), targetX - pose.getX());

                // 2. Calculate the ERROR (Difference between target and current)
                // follower.getPose().getHeading() returns radians
                double angleError = targetAngle - pose.getHeading();

                // 3. Normalize the error so the robot takes the shortest path (don't spin 300 degrees)
                while (angleError > Math.PI) angleError -= 2 * Math.PI;
                while (angleError < -Math.PI) angleError += 2 * Math.PI;

                // 4. Convert error to rotation power (Proportional control)
                rx = angleError * turnP;

                // 5. Optional: Cap the speed so it doesn't spin too violently
                rx = Range.clip(rx, -0.5, 0.5);

                // 6. If the error is tiny, stop aiming
                if (Math.abs(angleError) < Math.toRadians(1)) {
                    // aiming = false; // Uncomment if you want it to shut off automatically
                }
            }
            if (!aiming){
                rx = -gamepad1.right_stick_x;
            }

            follower.setTeleOpDrive(-y, x, rx, true);
//
//                follower.turnToPoint();
//                double angleToPoint = Math.atan2(targetY - follower.getPose().getY(), targetX - follower.getPose().getX());
//                follower.holdPoint(new BezierPoint(follower.getPose().getX(),follower.getPose().getY()), angleToPoint);
//            }

//
//            if (follower.isBusy()) {
//                // If holding a point (Auto-Aligning), we let the follower handle rotation (0)
//                // but we still allow the driver to strafe and drive with the left stick
//                follower.setTeleOpDrive(y, x, 0, true);
//            } else {
//                // Normal TeleOp driving
//                follower.setTeleOpDrive(y, x, rx, true);
//            }
//
//            if (Math.abs(rx) > 0.2) {
//                follower.breakFollowing();
//            }

            // OUTTAKE
            if(gamepad2.left_bumper) {
                targetOuttakeVelocity = FAR_OUTTAKE_VELOCITY;
                autoUpdate = false;
            } else if(gamepad2.right_bumper) {
                targetOuttakeVelocity = CLOSE_OUTTAKE_VELOCITY;
                autoUpdate = false;
            } else if (gamepad2.dpad_right) {
                targetOuttakeVelocity = 0;
                autoUpdate = false;
            } else if (gamepad2.dpad_up){
                autoUpdate = true;
                velocityTimer.reset();
            }
            outtake1.setVelocity(targetOuttakeVelocity);
            outtake2.setVelocity(targetOuttakeVelocity);


            // INTAKE
            if (gamepad1.right_bumper) {
                intake1Power = CLOSE_INTAKE_POWER;
                intakeStatus = INTAKE_STATUS.INTAKE_STARTED;
                intake2Power = 0;
//                blockerPosition = JR_OUTTAKE_BLOCK;
            } else if (gamepad1.left_bumper) {
                intake1Power = 0;
                intake2Power = 0;
                intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
            } else if (gamepad1.dpad_left) {
                intake1Power = -CLOSE_INTAKE_POWER;
            } else if (gamepad2.a){
                intake2Power = 1.0;
                intake1Power = CLOSE_INTAKE_POWER;
                intakeStatus = INTAKE_STATUS.INTAKE_STARTED;
                blockerPosition = JR_OUTTAKE_OPEN;
            } else if (gamepad2.b){
                intake2Power = 0.0;
                intake1Power = 0;
                intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
            }
            blocker.setPosition(Range.clip(blockerPosition,0,1));

            intake1Vel = intake1.getVelocity();

            if (intakeStatus == INTAKE_STATUS.INTAKE_STARTED && intake1Vel > 800) {
                intakeStatus = INTAKE_STATUS.INTAKE_NEED_TO_BE_MONITORED;
            }
            else if (intakeStatus == INTAKE_STATUS.INTAKE_NEED_TO_BE_MONITORED && intake1Vel < 200) {
                intakeStatus = INTAKE_STATUS.INTAKE_MONITORING;
                intakeTimer.reset();
            }
            else if (intakeStatus == INTAKE_STATUS.INTAKE_MONITORING && intake1Vel < 400 && intakeTimer.milliseconds() > 50) {
                intakeStatus = INTAKE_STATUS.INTAKE_JAMMED;
            }
            else if (intakeStatus == INTAKE_STATUS.INTAKE_JAMMED) {
                intake1Power = 0;
                intakeTimer.reset();
                intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
            }


            intake1Vel = intake1.getVelocity();

            intake1.setPower(intake1Power);
            intake2.setPower(intake2Power);
            telemetry();

            telemetry.update();
        }

    }


    public void initHardware() {
        initMotorOne(kP, kI, kD, F, position);
        initMotorTwo(kP, kI, kD, F, position);
//        initDriveMotors();
//        initCamera();
        initIntake();
    }

    public void turnToPoint(){

    }
    public void initDriveMotors(){
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightBack = hardwareMap.get(DcMotor.class, "rr");
        leftBack = hardwareMap.get(DcMotor.class, "lr");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initIntake(){
        intake1 = hardwareMap.get(DcMotorEx.class,"intake1");
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2 = hardwareMap.get(DcMotor.class,"intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blocker = hardwareMap.get(Servo.class,"blocker");
        blocker.setPosition(Range.clip(blockerPosition,0,1));
    }
    private void initCamera(){
//        limelight3A = hardwareMap.get(Limelight3A.class,"limelight3A");
//        limelight3A.start();
    }

//    private double getRobotToGoalDistance(){
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//        double distance = -1.0;
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                if (detection.id == 20 || detection.id == 24){
//                    distance = detection.ftcPose.y;
//                    break;
//                }
//            }
//        }
//        return distance;
//    } // end method

//    private boolean getGoal(){
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
////        double id = -1.0;
//        boolean goal = false;
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                if (detection.id == 20 || detection.id == 24){
//                    goal = true;
//                    break;
//                }
//            }
//        }
//        return goal;
//    }

    private void initMotorOne(double kP, double kI, double kD, double F, double position) {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setDirection(DcMotorEx.Direction.FORWARD);
        outtake1.setPower(outtakeZeroPower);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake1.setVelocity(targetOuttakeVelocity);
    }

    private void initMotorTwo(double kP, double kI, double kD, double F, double position) {
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2.setDirection(DcMotorEx.Direction.FORWARD);
        outtake2.setPower(outtakeZeroPower);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setVelocity(targetOuttakeVelocity);
    }

    public void telemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Intake Power", "Intake Power: " + intake1Power);
        telemetry.addData("Target Velocity", targetOuttakeVelocity);
        telemetry.addData("Intake state", intakeStatus);
        telemetry.addData("intake velocity", intake1Vel);
        telemetry.addData("Intake TIMER",intakeTimer.milliseconds());
        telemetry.addData("Outtake 1 power", outtake1.getPower());
        telemetry.addData("Outtake 2 power", outtake2.getPower());
        telemetry.addData("Outtake 1 Velocity", outtake1.getVelocity());
        telemetry.addData("Outtake 2 Velocity", outtake2.getVelocity());
        telemetry.addData("blocker", blockerPosition);
    }
}

