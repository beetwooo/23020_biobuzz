package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Every shooter setting in one place. All of these can be changed live in Panels,
 * but Panels changes are lost when the robot restarts: copy good values back here.
 */
@Configurable
public class ShooterConstants {

    // ===== Hardware =====
    public static String FLYWHEEL_MOTOR = "SL";
    public static boolean FLYWHEEL_REVERSED = false;

    // Encoder ticks per motor revolution (28 for goBILDA / REV bare motors).
    public static double TICKS_PER_REV = 28;
    // Highest flywheel speed the code will ask for.
    public static double MAX_RPM = 5800;

    // ===== Velocity PIDF (runs on the REV hub) =====
    // Tune these yourself with the "Flywheel PID Tuner" OpMode.
    // While ALL four are 0, the hub's built-in default velocity PIDF is used instead
    // (otherwise a 0 PIDF would never spin the flywheel).
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    // ===== Ready check =====
    // Flywheel is "ready" after staying within READY_TOLERANCE_RPM for READY_TIME seconds.
    public static double READY_TOLERANCE_RPM = 150;
    public static double READY_TIME = 0.15;

    // ===== Firing window =====
    // After a shot starts, keep feeding this many seconds even if the RPM dips.
    // Your old 23020 code used 0.35. Raise it if balls get stuck halfway,
    // lower it if two balls go out in one window.
    public static double FIRE_TIME = 0.35;

    // ===== Distance -> speed =====
    // Shoot from two distances (inches from the goal), find the RPM that scores, write it here.
    // Speeds in between are blended; closer/further than these use the nearest value.
    public static double NEAR_DISTANCE = 24;
    public static double NEAR_RPM = 0;      // <- fill in
    public static double FAR_DISTANCE = 72;
    public static double FAR_RPM = 0;       // <- fill in

    // Used until NEAR_RPM and FAR_RPM are filled in.
    public static double DEFAULT_RPM = 3000;
}
