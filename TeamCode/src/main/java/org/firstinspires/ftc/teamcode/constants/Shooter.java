package org.firstinspires.ftc.teamcode.constants;

public class Shooter {
    public static double TURRET_P = 0;
    public static double TURRET_I = 0;
    public static double TURRET_D = 0;
    public static double TURRET_F = 0;

    public static double TURRET_DEADBAND_RADIANS = Math.toRadians(2);
    public static double TURRET_MAX_POWER = 0.7;
    public static double TURRET_INTEGRAL_LIMIT = 0.5;
    public static double TURRET_MAX_PID_DT_SECONDS = 0.1;
    public static double TURRET_MIN_ANGLE_RADIANS = Math.toRadians(-180);
    public static double TURRET_MAX_ANGLE_RADIANS = Math.toRadians(180);

    public static String TURRET_LEFT_SERVO_NAME = "TL";
    public static String TURRET_RIGHT_SERVO_NAME = "TR";
    public static String TURRET_ENCODER_NAME = "Tencoder";

    // Servo rotations per one turret rotation.
    public static double TURRET_GEAR_RATIO = (30.0/60.0)*(105.0/25.0);
    public static double TURRET_ENCODER_DIRECTION = 1; //반시계방향으로 회전시 각도 증가
    public static double TURRET_LEFT_POWER_DIRECTION = -1;
    public static double TURRET_RIGHT_POWER_DIRECTION = -1;

}
