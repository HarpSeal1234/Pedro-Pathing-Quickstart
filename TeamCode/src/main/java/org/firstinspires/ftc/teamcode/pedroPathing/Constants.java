package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class Constants {
    // Robot dimensions (inches) — 240mm x 240mm
    public static final double ROBOT_WIDTH_INCHES = 9.45;
    public static final double ROBOT_LENGTH_INCHES = 9.45;
    public static final double ROBOT_HALF_WIDTH = ROBOT_WIDTH_INCHES / 2.0;

    public static final double BLUE_GOAL_POSITION_X = 0;
    public static final double BLUE_GOAL_POSITION_Y = 144;
    public static final double RED_GOAL_POSITION_X = 144;
    public static final double RED_GOAL_POSITION_Y = 144;

    // Minimum distance from goal to allow shooting (inches)
    public static final double MIN_SHOOT_DISTANCE = 45;

    // Maximum turret rotation in degrees (left or right from center)
    public static final double MAX_TURRET_ANGLE = 135;

    // Outtake must reach this fraction of target velocity before intake2 feeds balls (0.0–1.0)
    public static final double OUTTAKE_SPEED_THRESHOLD = 0.90;

    // Near launch zone triangle vertices (inches)
    public static final double NEAR_LAUNCH_ZONE_X1 = 0;
    public static final double NEAR_LAUNCH_ZONE_Y1 = 144;
    public static final double NEAR_LAUNCH_ZONE_X2 = 72;
    public static final double NEAR_LAUNCH_ZONE_Y2 = 72;
    public static final double NEAR_LAUNCH_ZONE_X3 = 144;
    public static final double NEAR_LAUNCH_ZONE_Y3 = 144;

    // Far launch zone triangle vertices (inches)
    public static final double FAR_LAUNCH_ZONE_X1 = 48;
    public static final double FAR_LAUNCH_ZONE_Y1 = 0;
    public static final double FAR_LAUNCH_ZONE_X2 = 72;
    public static final double FAR_LAUNCH_ZONE_Y2 = 48;
    public static final double FAR_LAUNCH_ZONE_X3 = 96;
    public static final double FAR_LAUNCH_ZONE_Y3 = 0;

    /**
     * Checks if a point (px, py) is inside the triangle defined by
     * (x1,y1), (x2,y2), (x3,y3) using the area (sign) method.
     */
    public static boolean isInsideTriangle(double px, double py,
                                           double x1, double y1,
                                           double x2, double y2,
                                           double x3, double y3) {
        double d1 = sign(px, py, x1, y1, x2, y2);
        double d2 = sign(px, py, x2, y2, x3, y3);
        double d3 = sign(px, py, x3, y3, x1, y1);

        boolean hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(hasNeg && hasPos);
    }

    private static double sign(double px, double py,
                               double x1, double y1,
                               double x2, double y2) {
        return (px - x2) * (y1 - y2) - (x1 - x2) * (py - y2);
    }

    /**
     * Checks if any corner of the robot is inside the near launch zone triangle.
     */
    public static boolean isInNearLaunchZone(double robotX, double robotY) {
        return isRobotAnyCornerInsideTriangle(robotX, robotY,
                NEAR_LAUNCH_ZONE_X1, NEAR_LAUNCH_ZONE_Y1,
                NEAR_LAUNCH_ZONE_X2, NEAR_LAUNCH_ZONE_Y2,
                NEAR_LAUNCH_ZONE_X3, NEAR_LAUNCH_ZONE_Y3);
    }

    /**
     * Checks if any corner of the robot is inside the far launch zone triangle.
     */
    public static boolean isInFarLaunchZone(double robotX, double robotY) {
        return isRobotAnyCornerInsideTriangle(robotX, robotY,
                FAR_LAUNCH_ZONE_X1, FAR_LAUNCH_ZONE_Y1,
                FAR_LAUNCH_ZONE_X2, FAR_LAUNCH_ZONE_Y2,
                FAR_LAUNCH_ZONE_X3, FAR_LAUNCH_ZONE_Y3);
    }

    /**
     * Checks if ANY corner of the robot is inside the given triangle.
     * Only one corner needs to be inside for the robot to be considered in the zone.
     */
    private static boolean isRobotAnyCornerInsideTriangle(double cx, double cy,
                                                  double x1, double y1,
                                                  double x2, double y2,
                                                  double x3, double y3) {
        double h = ROBOT_HALF_WIDTH;
        return isInsideTriangle(cx - h, cy - h, x1, y1, x2, y2, x3, y3)
            || isInsideTriangle(cx + h, cy - h, x1, y1, x2, y2, x3, y3)
            || isInsideTriangle(cx - h, cy + h, x1, y1, x2, y2, x3, y3)
            || isInsideTriangle(cx + h, cy + h, x1, y1, x2, y2, x3, y3);
    }

    /**
     * Checks if the robot is inside either launch zone.
     */
    public static boolean isInLaunchZone(double robotX, double robotY) {
        return isInNearLaunchZone(robotX, robotY) || isInFarLaunchZone(robotX, robotY);
    }

    /**
     * 0.3467: -90
     * 0.6667: 90
     */
    public static final double TURRET_POSITION_PER_DEGREE =0.0017777777777777779;


    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-31.6)
            .lateralZeroPowerAcceleration(-62.15)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.08,
                    0,
                    0.001,
                    0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.5,
                    0.0,
                    0.001,
                    0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0.0,
                    0.000045,
                    0.6,
                    0.015))
            .centripetalScaling(0.0007)
            .mass(14.47);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .xVelocity(81.7)
            .yVelocity(59)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.8)
            .strafePodX(-6.75)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}

