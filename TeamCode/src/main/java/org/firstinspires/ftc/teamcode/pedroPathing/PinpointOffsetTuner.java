package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Pinpoint Offset Tuner
 *
 * This OpMode helps you calibrate forwardPodY and strafePodX in your
 * Pinpoint localizer constants (Constants.java).
 *
 * HOW TO USE:
 * 1. Place the robot on a smooth surface
 * 2. Run this OpMode
 * 3. Use the left stick X to spin the robot slowly in place
 *    (or manually spin it by hand)
 * 4. Spin the robot several full rotations (5-10 is ideal)
 * 5. Watch the telemetry:
 *    - If X drifts → adjust strafePodX in Constants.java
 *    - If Y drifts → adjust forwardPodY in Constants.java
 * 6. Press A to reset the starting position
 *
 * TUNING TIPS:
 * - If X increases when spinning CCW → make strafePodX more negative
 * - If X decreases when spinning CCW → make strafePodX more positive (less negative)
 * - If Y increases when spinning CCW → make forwardPodY more negative
 * - If Y decreases when spinning CCW → make forwardPodY more positive (less negative)
 *
 * Current offsets are shown on screen so you know what you're tuning from.
 */
@TeleOp(name = "Pinpoint Offset Tuner", group = "Tuning")
public class PinpointOffsetTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(0)));

        double startX = 72;
        double startY = 72;
        int rotationCount = 0;
        double lastHeading = 0;
        double totalHeadingChange = 0;


        // Current offsets from Constants.java
        // forwardPodY = -2.8, strafePodX = -6.75
        telemetry.addLine("=== PINPOINT OFFSET TUNER ===");
        telemetry.addLine("Spin the robot in place using left stick X");
        telemetry.addLine("Watch X Drift and Y Drift on telemetry");
        telemetry.addLine("");
        telemetry.addLine("Current offsets (from Constants.java):");
        telemetry.addData("  forwardPodY", -2.8);
        telemetry.addData("  strafePodX", -6.5);
        telemetry.addLine("");
        telemetry.addLine("Press A to reset position");
        telemetry.update();

        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            // Spin the robot using left stick X
            double turnPower = gamepad1.left_stick_x * 0.4; // slow spin for accuracy
            follower.setTeleOpDrive(0, 0, turnPower);
            follower.update();

            Pose pose = follower.getPose();

            // Track total rotation
            double currentHeading = pose.getHeading();
            double headingDelta = currentHeading - lastHeading;
            // Normalize delta
            while (headingDelta > Math.PI) headingDelta -= 2 * Math.PI;
            while (headingDelta < -Math.PI) headingDelta += 2 * Math.PI;
            totalHeadingChange += headingDelta;
            lastHeading = currentHeading;

            rotationCount = (int) (Math.abs(totalHeadingChange) / (2 * Math.PI));

            double xDrift = pose.getX() - startX;
            double yDrift = pose.getY() - startY;
            double totalDrift = Math.sqrt(xDrift * xDrift + yDrift * yDrift);

            // Reset on A press
            if (gamepad1.a) {
                startX = pose.getX();
                startY = pose.getY();
                totalHeadingChange = 0;
            }

            telemetry.addLine("=== PINPOINT OFFSET TUNER ===");
            telemetry.addLine("");
            telemetry.addData("Full Rotations", rotationCount);
            telemetry.addData("Total Heading Change (deg)", String.format("%.1f", Math.toDegrees(totalHeadingChange)));
            telemetry.addLine("");
            telemetry.addLine("--- DRIFT (should be ~0) ---");
            telemetry.addData("X Drift (inches)", String.format("%.3f", xDrift));
            telemetry.addData("Y Drift (inches)", String.format("%.3f", yDrift));
            telemetry.addData("Total Drift (inches)", String.format("%.3f", totalDrift));
            telemetry.addLine("");

            if (rotationCount >= 1) {
                telemetry.addData("X Drift per rotation", String.format("%.3f", xDrift / rotationCount));
                telemetry.addData("Y Drift per rotation", String.format("%.3f", yDrift / rotationCount));
                telemetry.addLine("");
            }

            telemetry.addLine("--- CURRENT POSE ---");
            telemetry.addData("X", String.format("%.3f", pose.getX()));
            telemetry.addData("Y", String.format("%.3f", pose.getY()));
            telemetry.addData("Heading (deg)", String.format("%.1f", Math.toDegrees(pose.getHeading())));
            telemetry.addLine("");
            telemetry.addLine("--- CURRENT OFFSETS (in Constants.java) ---");
            telemetry.addData("forwardPodY", -2.8);
            telemetry.addData("strafePodX", -6.75);
            telemetry.addLine("");
            telemetry.addLine("If X drifts -> adjust strafePodX");
            telemetry.addLine("If Y drifts -> adjust forwardPodY");
            telemetry.addLine("Press A to reset");
            telemetry.update();
        }
    }
}
