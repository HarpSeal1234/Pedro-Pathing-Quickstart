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

public class Constants {
    public static final PIDFCoefficients farPidfCoefficients = new PIDFCoefficients(1.8, 0.0002, 0.0, 14.3);
    public static final PIDFCoefficients closePidfCoefficients = new PIDFCoefficients(1.8, 0.0002, 0.0, 14.2);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-31.6)
            .lateralZeroPowerAcceleration(-62.15)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.08,
                    0,
                    0.001,
                    0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.0,
                    0.0,
                    0.01,
                    0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0.0,
                    0.000045,
                    0.85,
                    0.015))
            .centripetalScaling(0.0007)
            .mass(14.47);

    // PathConstraints(tValue, velocity, translational, heading, timeout, brakingStrength, searchLimit, brakingStart)
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 0.1, 0.1, 0.003, 200, 1, 10, 1);

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
            .strafePodX(-6.5)
            .yawScalar(0.99811)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}

