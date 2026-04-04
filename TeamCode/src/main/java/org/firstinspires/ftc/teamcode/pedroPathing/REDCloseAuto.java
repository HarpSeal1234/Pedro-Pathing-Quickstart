package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//@Disabled
@Autonomous(name = "Red Close Auto", group = "OrcaRobotics")
public class RedCloseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer,waitTimer;

    private int pathState;
    private double launchTime = 1000;

    private double grabTime = 1400;
    private double pickupSpeed = 0.8;
    private double grabSpeed = 0.6;

    private Launcher launcher;
    private final Pose startPose = new Pose(15.5, 112.5, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose1 = new Pose(57, 86, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1PoseStart = new Pose(40, 55, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1PoseEnd = new Pose(10, 55, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose scorePose2 = new Pose(57, 81, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose openGateGrabStartPose = new Pose(15, 60, Math.toRadians(160)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose openGateGrabEndPose = new Pose(10, 60, Math.toRadians(160)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose scorePose3 = new Pose(57, 83, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup2PoseStart = new Pose(40, 82, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2PoseEnd = new Pose(19, 82, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose scorePose4 = new Pose(57, 83, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose openGateGrabPose2 = new Pose(11.5, 60.5, Math.toRadians(160)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose scorePose5 = new Pose(57, 83, Math.toRadians(180));
    private final Pose scorePose6 = new Pose(57, 83, Math.toRadians(180));
    private final Pose leavePose = new Pose(36, 70, Math.toRadians(180));


    private PathChain
            score1,
            pickup1,
            pickup1Path,
            score2,
            openGateStartGrab,
            openGateEndGrab,
            score3,
            openGateStartGrab2,
            openGateEndGrab2,
            score4,
            openGateStartGrab3,
            openGateEndGrab3,
            score5,
            pickup2,
            pickup2Path,
            score6,
            leave;

    public void buildPaths() {
        // score1: Start launcher mid-path so it's ready by arrival
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0.0, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_NEAR))
                .build();

        // pickup1: Switch to pickup mode mid-path
        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose1, new Pose(56, 64), pickup1PoseStart))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup1Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseStart, pickup1PoseEnd))
                .build();

        // score2
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1PoseEnd, new Pose(54, 54), scorePose2))
                .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), scorePose2.getHeading())
                .addParametricCallback(0.3, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_NEAR))
                .build();

        // openGateStartGrab: Fast approach to gate area
        openGateStartGrab = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose2, new Pose(48, 62), openGateGrabStartPose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), openGateGrabStartPose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();
        // openGateEndGrab: Slow grab into gate
        openGateEndGrab = follower.pathBuilder()
                .addPath(new BezierLine(openGateGrabStartPose, openGateGrabEndPose))
                .setConstantHeadingInterpolation(openGateGrabStartPose.getHeading())
                .build();

        // score3: from gate grab end to score3
        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(openGateGrabEndPose, new Pose(51, 54), scorePose3))
                .setLinearHeadingInterpolation(openGateGrabEndPose.getHeading(), scorePose3.getHeading())
                .addParametricCallback(0.3, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_NEAR))
                .build();

        // openGateStartGrab2: Fast approach to gate area (second time)
        openGateStartGrab2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose3, new Pose(47, 68), openGateGrabStartPose))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), openGateGrabStartPose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();
        // openGateEndGrab2: Slow grab into gate (second time)
        openGateEndGrab2 = follower.pathBuilder()
                .addPath(new BezierLine(openGateGrabStartPose, openGateGrabEndPose))
                .setConstantHeadingInterpolation(openGateGrabStartPose.getHeading())
                .build();

        // score4: from gate grab end to score4
        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(openGateGrabEndPose, new Pose(42, 82), scorePose4))
                .setLinearHeadingInterpolation(openGateGrabEndPose.getHeading(), scorePose4.getHeading())
                .addParametricCallback(0.3, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_NEAR))
                .build();

        // openGateStartGrab3: Fast approach to gate area (third time)
        openGateStartGrab3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose4, new Pose(47, 68), openGateGrabStartPose))
                .setLinearHeadingInterpolation(scorePose4.getHeading(), openGateGrabStartPose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();
        // openGateEndGrab3: Slow grab into gate (third time)
        openGateEndGrab3 = follower.pathBuilder()
                .addPath(new BezierLine(openGateGrabStartPose, openGateGrabEndPose))
                .setConstantHeadingInterpolation(openGateGrabStartPose.getHeading())
                .build();
        // score5: from gate grab end to score5
        score5 = follower.pathBuilder()
                .addPath(new BezierCurve(openGateGrabEndPose, new Pose(42, 82), scorePose5))
                .setLinearHeadingInterpolation(openGateGrabEndPose.getHeading(), scorePose5.getHeading())
                .addParametricCallback(0.3, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_NEAR))
                .build();

        // pickup2: Switch to pickup mode mid-path (from score5)
        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose5, new Pose(47, 83), pickup2PoseStart))
                .setLinearHeadingInterpolation(scorePose5.getHeading(), pickup2PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup2Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseStart, pickup2PoseEnd))
                .build();

        // score6
        score6 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseEnd, new Pose(42, 82), scorePose6))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose6.getHeading())
                .addParametricCallback(0.3, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_NEAR))
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose6, new Pose(37, 87), leavePose))
                .setLinearHeadingInterpolation(scorePose6.getHeading(), leavePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // === SCORE 1 ===
            case 0:
                follower.followPath(score1, 1, true);
                setPathState(1);
                break;
            case 1: // Arrived → wait for flywheel to reach speed
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2: // Wait for flywheel to reach target velocity
                if (launcher.isFlywheelReady()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(3);
                }
                break;
            case 3: // Wait for all balls
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup1, 1, true);
                    setPathState(4);
                }
                break;

            // === PICKUP 1 ===
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pickup1Path, pickupSpeed, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(score2, 1, true);
                    setPathState(6);
                }
                break;

            // === SCORE 2 (callback preps launcher at 30%) ===
            case 6: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(7);
                }
                break;
            case 7: // Wait for all balls
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(openGateStartGrab, 1.0, true);
                    setPathState(8);
                }
                break;

            // === OPEN GATE GRAB 1 ===
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(openGateEndGrab, grabSpeed, true);
                    setPathState(9);
                }
                break;
            case 9: // Wait for pickup
                if (!follower.isBusy() && actionTimer.getElapsedTime() > grabTime) {
                    follower.followPath(score3, 1.0, true);
                    setPathState(10);
                }
                break;

            // === SCORE 3 (callback preps launcher at 30%) ===
            case 10: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(11);
                }
                break;
            case 11: // Wait for all balls
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(openGateStartGrab2, 1.0, true);
                    setPathState(12);
                }
                break;

            // === OPEN GATE GRAB 2 ===
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(openGateEndGrab2, grabSpeed, true);
                    setPathState(13);
                }
                break;
            case 13: // Wait for pickup
                if (!follower.isBusy() && actionTimer.getElapsedTime() > grabTime) {
                    follower.followPath(score4, 1.0, true);
                    setPathState(14);
                }
                break;

            // === SCORE 4 (callback preps launcher at 30%) ===
            case 14: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(15);
                }
                break;
            case 15: // Wait for all balls
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(openGateStartGrab3, 1.0, true);
                    setPathState(16);
                }
                break;

            // === OPEN GATE GRAB 3 ===
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(openGateEndGrab3, grabSpeed, true);
                    setPathState(17);
                }
                break;
            case 17: // Wait for pickup
                if (!follower.isBusy() && actionTimer.getElapsedTime() > grabTime) {
                    follower.followPath(score5, 1.0, true);
                    setPathState(18);
                }
                break;

            // === SCORE 5 (callback preps launcher at 30%) ===
            case 18: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(19);
                }
                break;
            case 19: // Wait for all balls
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup2, 1, false);
                    setPathState(20);
                }
                break;

            // === PICKUP 2 ===
            case 20:
                if (!follower.isBusy()) {
                    follower.followPath(pickup2Path, pickupSpeed, true);
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    follower.followPath(score6, 1, true);
                    setPathState(22);
                }
                break;

            // === SCORE 6 (callback preps launcher at 30%) ===
            case 22: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(23);
                }
                break;
            case 23: // Wait for all balls, then leave
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(leave, 1, true);
                    setPathState(24);
                }
                break;

            // === LEAVE ===
            case 24:
                if (!follower.isBusy()) {
                    setPathState(-1);
                    requestOpModeStop();
                }
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        launcher.update();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        waitTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        launcher = new Launcher(hardwareMap, Constants.closePidfCoefficients);

        buildPaths();
        follower.setStartingPose(startPose);
//        stop();
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
