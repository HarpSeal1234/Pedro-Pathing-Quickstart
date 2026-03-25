package org.firstinspires.ftc.teamcode.pedroPathing.NotUSING; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Example Auto", group = "Examples")
public class auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(48)); // Start Pose of our robot.
    private final Pose scorePose1 = new Pose(-24, -24, Math.toRadians(48)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose2 = new Pose(-24, -24, Math.toRadians(48)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1PoseStart = new Pose(-36, -10, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1PoseEnd = new Pose(-36, -5, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose pickup2PoseStart = new Pose(-36, -10, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose pickup2PoseEnd = new Pose(-36, -5, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose scorePose3 = new Pose(-24, -24, Math.toRadians(48)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    private PathChain score1,pickup1, score2;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build();
        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, pickup1PoseStart))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1PoseStart.getHeading())
                .addPath(new BezierLine(pickup1PoseStart, pickup1PoseEnd))
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1PoseEnd,new Pose(-16,-10),scorePose2))
                .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), scorePose2.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to score 1
                follower.followPath(score1,1,true);
                setPathState(1);
                break;

            case 1: // Wait until robot finishes score1, then move to pickup
                if (!follower.isBusy()) {
                    follower.followPath(pickup1,0.5,true);
                    setPathState(2);
                }
                break;

            case 2: // Wait until robot finishes pickup1, then move to score 2
                if (!follower.isBusy()) {
                    follower.followPath(score2, 1, true);
                    setPathState(3);
                }
                break;

            case 3: // Done
                if (!follower.isBusy()) {
                    // Auto finished!
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

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
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
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