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
@Autonomous(name = "BlueFarAuto", group = "Examples")
public class BlueFarAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer,waitTimer;

    private int pathState;
    private double launchTime = 1500;
    private double pickupSpeed = 0.8;

    private Launcher launcher;
    private final Pose startPose = new Pose(55.8, 7.5, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose pickup1PoseStart = new Pose(36, 36, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1PoseEnd = new Pose(14, 36, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose scorePose2 = new Pose(55.8, 81, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup2PoseStart = new Pose(11, 10, Math.toRadians(190)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose scorePose3 = new Pose(55.8, 81, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup3PoseStart = new Pose(11, 10, Math.toRadians(190)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose scorePose5 = new Pose(55.8, 81, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose leavePose = new Pose(44, 25, Math.toRadians(180))  ; // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    private PathChain
            pickup1,
            pickup1Path,
            score2,
            pickup2,
            score3,
            pickup3,
            score4,
            leave;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,new Pose(56,26),pickup1PoseStart))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1PoseStart.getHeading())
                .build();
        pickup1Path  = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseStart, pickup1PoseEnd))
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1PoseEnd,new Pose(48,38),scorePose2))
                .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), scorePose2.getHeading())
                .build();
        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose2,new Pose(38,20),pickup2PoseStart))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup2PoseStart.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseStart,new Pose(29,21), scorePose3))
                .setLinearHeadingInterpolation(pickup2PoseStart.getHeading(), scorePose3.getHeading())
                .build();
        pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose3,new Pose(47,68), pickup3PoseStart))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), pickup3PoseStart.getHeading())
                .build();
        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3PoseStart,new Pose(29,21),scorePose5))
                .setLinearHeadingInterpolation(pickup3PoseStart.getHeading(), scorePose5.getHeading())
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose5,new Pose(54,21),leavePose))
                .setLinearHeadingInterpolation(scorePose5.getHeading(), leavePose.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to score 1
                launcher.setState(Launcher.LauncherState.START_LAUNCHING_FAR);
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
                        follower.followPath(pickup2, 0.5, true);
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
                if (actionTimer.getElapsedTime() > 1600) {
                    follower.followPath(score3, 1, true);
//                    launcher.setState(Launcher.LauncherState.IDLE);
                    setPathState(6);
                }
                break;


            case 6: // Wait until robot finishes pickup1, then move to score 2
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(7);
                }
                break;

            case 7: // Wait for launch 3
                if (actionTimer.getElapsedTime() > launchTime) {
                    launcher.setState(Launcher.LauncherState.PICKUP);
                    follower.followPath(pickup3, 1, true);
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
                        setPathState(1101);
                    }
                }
                break;

            case 10:
                if (!follower.isBusy()){
                    if (actionTimer.getElapsedTime() > launchTime) {
                        follower.followPath(pickup3, 0.6, true);
                        launcher.setState(Launcher.LauncherState.PICKUP);
//                        launcher.setState(Launcher.LauncherState.IDLE);
                        setPathState(1001);
                    }
                }
                break;

            case 1001: // Wait a moment for pickup, then drive to score 3
                if (actionTimer.getElapsedTime() > 1200) {
                    follower.followPath(score4, 1, true);
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
//        launcher.updateTurret(follower.getPose());
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