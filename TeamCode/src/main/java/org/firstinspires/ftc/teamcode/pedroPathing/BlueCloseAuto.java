package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Visualizer Auto", group = "Examples")
public class BlueCloseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer,waitTimer;

    private int pathState;
    private double launchTime = 1500;
    private double pickupSpeed = 0.8;

    private Launcher launcher;
    private final Pose startPose = new Pose(22, 120, Math.toRadians(138.5)); // Start Pose of our robot.
    private final Pose scorePose1 = new Pose(51, 93, Math.toRadians(138.5)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1PoseStart = new Pose(39, 60, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1PoseEnd = new Pose(13, 60, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose scorePose2 = new Pose(51, 93, Math.toRadians(138.5)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose openGateGrabPose = new Pose(12, 58, Math.toRadians(145)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose scorePose3 = new Pose(51, 93, Math.toRadians(138.5)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup2PoseStart = new Pose(39, 81, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2PoseEnd = new Pose(19, 81, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose scorePose4 = new Pose(51, 93, Math.toRadians(138.5)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose openGateGrabPose2 = new Pose(12, 58, Math.toRadians(145)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose scorePose5 = new Pose(51, 93, Math.toRadians(138.5)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose leavePose = new Pose(25, 99, Math.toRadians(138.5)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    private PathChain
            score1,
            pickup1,
            pickup1Path,
            score2,
            openGateGrab,
            score3,
            pickup2,
            pickup2Path,
            score4,
            openGateGrab2,
            score5,
            leave;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build();
        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose1,new Pose(56,64),pickup1PoseStart))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1PoseStart.getHeading())
                .build();
        pickup1Path  = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseStart, pickup1PoseEnd))
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1PoseEnd,new Pose(54,54),scorePose2))
                .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), scorePose2.getHeading())
                .build();
        openGateGrab = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose2,new Pose(48,62), openGateGrabPose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), openGateGrabPose.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(openGateGrabPose,new Pose(51,54), scorePose3))
                .setLinearHeadingInterpolation(openGateGrabPose.getHeading(), scorePose3.getHeading())
                .build();
        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose3,new Pose(47,83),pickup2PoseStart))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), pickup2PoseStart.getHeading())
                .build();
        pickup2Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseStart, pickup2PoseEnd))
                .build();
        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseEnd,new Pose(42,82),scorePose4))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose4.getHeading())
                .build();
        openGateGrab2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose4,new Pose(47,68), openGateGrabPose2))
                .setLinearHeadingInterpolation(scorePose4.getHeading(), openGateGrabPose2.getHeading())
                .build();
        score5 = follower.pathBuilder()
                .addPath(new BezierCurve(openGateGrabPose,new Pose(51,93),scorePose5))
                .setLinearHeadingInterpolation(openGateGrabPose.getHeading(), scorePose5.getHeading())
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose5,new Pose(37,87),leavePose))
                .setLinearHeadingInterpolation(scorePose5.getHeading(), leavePose.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to score 1
                follower.followPath(score1,1,true);
                launcher.setState(Launcher.LauncherState.START_LAUNCHING_NEAR);
                setPathState(1);
                break;

            case 1: // Wait until arrived at score1, then LAUNCH
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() > 500) {
                        launcher.setState(Launcher.LauncherState.LAUNCH);
                        setPathState(101);
                    }
                }
                break;

            case 101: // Wait for launch to finish
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup1, 1, true);
                    launcher.setState(Launcher.LauncherState.PICKUP);
                    setPathState(2);
                }
                break;

            case 2: // Wait until robot finishes pickup1, then move to score 2
                if (!follower.isBusy()) {
                    follower.followPath(pickup1Path, pickupSpeed, true);
                    setPathState(3);
                }
                break;

            case 3: // Wait until robot finishes pickup1, then move to score 2
                if (!follower.isBusy()) {
                    follower.followPath(score2, 1, true);
//                    launcher.setState(Launcher.LauncherState.IDLE);
                    setPathState(4);
                }
                break;

            case 4: // Arrived at score 2? Now LAUNCH.
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(401);
                }
                break;

            case 401:
                if (!follower.isBusy()){
                    if (actionTimer.getElapsedTime() > launchTime) {
                        follower.followPath(openGateGrab, 0.8, true);
                        launcher.setState(Launcher.LauncherState.PICKUP);
//                        launcher.setState(Launcher.LauncherState.IDLE);
                        setPathState(5);
                    }
                }
                break;

            case 5: // Wait until robot finishes pickup1, then move to score 2
                if (!follower.isBusy()) {
                    setPathState(501);
                }
                break;

            case 501: // Wait a moment for pickup, then drive to score 3
                if (actionTimer.getElapsedTime() > 1200) {
                    follower.followPath(score3, 1, true);
//                    launcher.setState(Launcher.LauncherState.IDLE);
                    setPathState(6);
                }
                break;


            case 6: // Wait until robot finishes pickup1, then move to score 2
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(601);
                }
                break;

            case 601: // Wait for launch 3
                if (actionTimer.getElapsedTime() > launchTime) {
                    launcher.setState(Launcher.LauncherState.PICKUP);
                    follower.followPath(pickup2, 1, true);
                    setPathState(7);
                }
                break;

            case 7: // Wait until robot finishes pickup1, then move to score 2
                if (!follower.isBusy()) {
                    follower.followPath(pickup2Path, pickupSpeed, true);
                    setPathState(8);
                }
                break;

            case 8: // Wait until robot finishes pickup1, then move to score 2
                if (!follower.isBusy()) {
                    follower.followPath(score4, 1, true);
//                    launcher.setState(Launcher.LauncherState.IDLE);
                    setPathState(9);
                }
                break;

            case 9: // Wait until robot finishes pickup1, then move to score 2
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    if (actionTimer.getElapsedTime() > launchTime) {
                        setPathState(10);
                    }
                }
                break;

            case 10:
                if (!follower.isBusy()){
                    if (actionTimer.getElapsedTime() > launchTime) {
                        follower.followPath(openGateGrab2, 0.7, true);
                        launcher.setState(Launcher.LauncherState.PICKUP);
//                        launcher.setState(Launcher.LauncherState.IDLE);
                        setPathState(1001);
                    }
                }
                break;

            case 1001: // Wait a moment for pickup, then drive to score 3
                if (actionTimer.getElapsedTime() > 1200) {
                    follower.followPath(score5, 1, true);
//                    launcher.setState(Launcher.LauncherState.IDLE);
                    setPathState(11);
                }
                break;


            case 11: // Wait until robot finishes pickup1, then move to score 2
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(1101);
                }
                break;

            case 1101: // Wait for launch 3
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(leave,1,true);
                    setPathState(100);
                }
                break;


            case 100: // Done
                if (!follower.isBusy()) {
                    // Auto finished!
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
        launcher = new Launcher(hardwareMap);

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