package org.firstinspires.ftc.teamcode.constants;

public class Field {

    public enum Goal{
        RED,
        BLUE
    }

    //pedro field 기준으로 (inches, origin = corner, field is 144 x 144)

    // Biobuzz: 4 goals, 2 per alliance, in the centre structure.
    // TOP = upper half of the field (y > 72), BOTTOM = lower half (y < 72).
    public static double RED_GOAL_X_TOP = 60;
    public static double RED_GOAL_Y_TOP = 84;
    public static double RED_GOAL_X_BOTTOM = 60;
    public static double RED_GOAL_Y_BOTTOM = 60;

    //===========================================//
    public static double BLUE_GOAL_X_TOP = 84;
    public static double BLUE_GOAL_Y_TOP = 84;
    public static double BLUE_GOAL_X_BOTTOM = 84;
    public static double BLUE_GOAL_Y_BOTTOM = 60;

    //===============================================\\

    // TeleOp starting pose (there is no autonomous). Place the robot here before INIT.
    // MEASURE these: (0, 0) means the robot is in the bottom-left corner.
    // If the pose is wrong, auto-aim aims at the wrong place.
    public static double RED_START_X = 0;
    public static double RED_START_Y = 0;
    public static double RED_START_HEADING_DEGREES = 90;

    public static double BLUE_START_X = 0;
    public static double BLUE_START_Y = 0;
    public static double BLUE_START_HEADING_DEGREES = 90;

    public static double BLUE_DRIVER_HEADING_DEGREES = 180;
    public static double RED_DRIVER_HEADING_DEGREES = 0;

    //=========================FLOWER=========================//

    public static double FLOWER_01_X = 48;
    public static double FLOWER_01_Y = 144;
    public static double FLOWER_02_X = 144;
    public static double FLOWER_02_Y = 96;
    public static double FLOWER_03_X = 96;
    public static double FLOWER_03_Y = 0;
    public static double FLOWER_04_X = 0;
    public static double FLOWER_04_Y = 48;

    //================RED_APPLY====================//

    public static double RED_APPLY_X = 7.3;
    public static double RED_APPLY_Y = 2.6;

    //===================RED_STOP====================//

    public static double RED_STOP_X = 14.1;
    public static double RED_STOP_Y = 91.3;

    //===================BLUE_APPLY====================//

    public static double BLUE_APPLY_X = 136.7;
    public static double BLUE_APPLY_Y = 141.4;

    //======================BLUE_STOP=====================//

    public static double BLUE_STOP_X = 129.9;
    public static double BLUE_STOP_Y = 52.7;
}


