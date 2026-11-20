
package org.firstinspires.ftc.teamcode.features.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Turns the whole robot so the shooter faces the goal, using Pedro 3's built-in
 * ManualDrive.headingLock() and the heading gain AutoTune measured for this robot
 * (Constants.foresightConfig.headingFeedback).
 *
 * Our part is only the goal angle: atan2(goalY - y, goalX - x).
 * Returns a Pedro-convention turn power (+ = counter-clockwise).
 */
@Configurable
public class ChassisAim {

    // Multiplies Pedro's AutoTuned heading gain. Lower it (e.g. 0.5) if the robot
    // swings past the goal and back; raise it if it turns too slowly.
    public static double GAIN_SCALE = 1.0;

    // Minimum turn power so the robot doesn't stall a few degrees short.
    // Raise until the robot just starts turning at small errors (try 0.05 - 0.12).
    public static double kS = 0.06;

    public static double MAX_TURN_POWER = 0.5;

    // Within this error the robot counts as aimed and stops turning.
    public static double TOLERANCE_DEGREES = 2.0;

    // Angle between robot forward and the shooter direction.
    // 0 = shooter faces straight out the front; Math.PI = out the back.
    public static double SHOOTER_MOUNT_OFFSET_RADIANS = 0;

    // Pedro's AutoTuned heading controller (from pedro/Constants.java).
    private final Controller tunedHeading = Constants.foresightConfig.headingFeedback.get();

    // What headingLock calls: Pedro gives (target heading, heading error) and expects a turn power.
    private final Controller aimController = (target, error) -> {
        if (Math.abs(error) <= Math.toRadians(TOLERANCE_DEGREES)) {
            return 0;   // aimed: stop turning
        }
        return GAIN_SCALE * tunedHeading.calculate(target, error)
                + kS * Math.signum(error);
    };

    private double targetHeading;
    private double headingError;
    private double turnPower;
    private boolean valid = false;

    /** Turn power that points the shooter at (goalX, goalY). Call once per loop. */
    public double calculate(Follower follower, double goalX, double goalY) {

        Pose robotPose = follower.pose();
        if (robotPose == null) {
            return none();
        }

        double deltaX = goalX - robotPose.x();
        double deltaY = goalY - robotPose.y();
        if (Math.hypot(deltaX, deltaY) < 0.001) {
            return none();
        }

        // The only goal-specific part: which way is the goal?
        targetHeading = Math.atan2(deltaY, deltaX) - SHOOTER_MOUNT_OFFSET_RADIANS;

        // Pedro wraps the error to the shortest turn and runs the controller.
        DrivePowers locked = ManualDrive.headingLock(
                follower, aimController, new DrivePowers(0, 0, 0), targetHeading);

        headingError = wrapAngle(targetHeading - robotPose.heading());
        turnPower = Range.clip(locked.turn(), -MAX_TURN_POWER, MAX_TURN_POWER);
        valid = true;

        return turnPower;
    }

    private double none() {
        valid = false;
        turnPower = 0;
        return 0;
    }

    private double wrapAngle(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians <= -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    // ===== Getters =====

    public double getTargetHeadingDegrees() {
        return Math.toDegrees(targetHeading);
    }

    public double getHeadingErrorDegrees() {
        return Math.toDegrees(headingError);
    }

    public double getTurnPower() {
        return turnPower;
    }

    /** True only when the last calculate() had a valid pose and the error is within tolerance. */
    public boolean isAimed() {
        return valid
                && Math.abs(headingError) <= Math.toRadians(TOLERANCE_DEGREES);
    }
}


/**
 * Setting	Start	Notes
 * GAIN_SCALE	1.0	Multiplies Pedro’s tuned gain. 6.67 is strong: it already hits the 0.5 power limit at about 4° of error. If the robot swings past the goal, try 0.5 and adjust from there.
 * kS	0.06	Raise it if the robot stops 1–3° short.
 * TOLERANCE_DEGREES	2	How close counts as aimed.
 * MAX_TURN_POWER	0.5	The fastest it can turn while aiming.
 * SHOOTER_MOUNT_OFFSET_RADIANS
 */
