package org.firstinspires.ftc.teamcode.constants;

public class Shooter {

    //pid
    public static double AIM_P = 0;
    public static double FLYWHEEL_P = 0;
    public static double FLYWHEEL_I = 0;
    public static double FLYWHEEL_D = 0;
    public static double FLYWHEEL_F = 0;

    public static String FLYWHEEL_NAME = "SL";

    /*
    public static double TURRET_MAX_POWER = 0.5;
    public static double TURRET_DEADBAND_RADIANS = 0;
    public static double TURRET_GEAR_RATIO;

     */

    //===========================================\\
        public static double FLYWHEEL_MAX_RPM = 5800;
        public static double FLYWHEEL_TPR = 28;

        // =====================================
        // FLYWHEEL HARDWARE
        // =====================================

        public static double WHEEL_RADIUS = 1.417;
        // Two 24T miter gears = 1:1
        public static double FLYWHEEL_MOTOR_REV_PER_WHEEL_REV = 1.0;

        // =====================================
        // READY CONDITION
        // =====================================
        // Ticks per second
        public static double FLYWHEEL_READY_TOLERANCE = 100;
        // Seconds
        public static double FLYWHEEL_READY_TIME = 0.15;
    }


