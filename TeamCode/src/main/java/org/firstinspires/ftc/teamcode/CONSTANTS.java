package org.firstinspires.ftc.teamcode;

public class CONSTANTS {
    public final static double DRIVE_POWER = 0.8;
    public final static double CLOSE_INTAKE_POWER = 0.9;
    public final static double JR_OUTTAKE_BLOCK = 0.78;
    public final static double JR_OUTTAKE_OPEN = 0.0;
    public final static int FAR_OUTTAKE_VELOCITY = 2200; // tip of far triangle: 2200, back part 2300
    public final static int CLOSE_OUTTAKE_VELOCITY = 1600;
    public final static double HOOD_MAX_POS = 0.92;
    public final static double HOOD_MIN_POS = 0.2;

    public final static double kP = 1.25;
    public final static double kI = 0.08;
    public final static double kD = 0.0;

    // Robot dimensions (inches) — 240mm x 240mm
    public static final double ROBOT_WIDTH_INCHES = 15.7;
    public static final double ROBOT_LENGTH_INCHES = 15.2;
    public static final double ROBOT_HALF_WIDTH = ROBOT_WIDTH_INCHES / 2.0;
    public static final double RED_GOAL_POSITION_X = 136; // 144 - 8
    public static final double RED_GOAL_POSITION_Y = 136; // 144 - 8
    public final static double BLUE_GOAL_POSITION_X = 5;
    public final static double BLUE_GOAL_POSITION_Y = 139;
    // Maximum turret rotation in degrees (left or right from center)
    public static final double MAX_TURRET_ANGLE = 135;
    public static final double TURRET_POSITION_PER_DEGREE =0.0017777777777777779;
}
