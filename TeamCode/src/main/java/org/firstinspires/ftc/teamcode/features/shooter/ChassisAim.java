
package org.firstinspires.ftc.teamcode.features.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

/**
 * Turns the whole robot so the shooter faces the goal.
 * Returns a Pedro-convention turn power (+ = counter-clockwise).
 * All values below can be tuned live in Panels.
 */
@Configurable
public class ChassisAim {

    // Turn power per radian of heading error.
    public static double kP = 1.5;

    // Damping: turn power per (rad/s) of rotation speed. Raise if the robot overshoots.
    public static double kD = 0.0;

    // Minimum turn power so the robot doesn't stall a few degrees short.
    // Raise until the robot just starts turning at small errors (try 0.05 - 0.12).
    public static double kS = 0.06;

    public static double MAX_TURN_POWER = 0.5;

    // Within this error the robot counts as aimed and stops turning.
    public static double TOLERANCE_DEGREES = 2.0;

    // Angle between robot forward and the shooter direction.
    // 0 = shooter faces straight out the front; Math.PI = out the back.
    public static double SHOOTER_MOUNT_OFFSET_RADIANS = 0;

    private double targetHeading;
    private double headingError;
    private double turnPower;
    private boolean valid = false;

    // ==========================================
    // AIM AT GOAL
    // ==========================================

    /**
     * @param angularVelocity robot rotation speed in rad/s (follower.velocity().omega), used for kD
     */
    public double calculate(
            Pose robotPose,
            double angularVelocity,
            double goalX,
            double goalY,
            double shotOffsetRadians) {

        if (robotPose == null) {
            return none();
        }

        // Robot-to-goal vector
        double deltaX = goalX - robotPose.x();
        double deltaY = goalY - robotPose.y();

        if (Math.hypot(deltaX, deltaY) < 0.001) {
            return none();
        }

        valid = true;

        // Field angle toward goal
        double fieldAngle = Math.atan2(deltaY, deltaX);

        // Required chassis heading
        targetHeading = wrapAngle(
                fieldAngle
                        - shotOffsetRadians
                        - SHOOTER_MOUNT_OFFSET_RADIANS
        );

        // Difference between desired and current heading
        headingError = wrapAngle(
                targetHeading - robotPose.heading()
        );

        // Stop rotating inside tolerance
        if (isAimed()) {
            turnPower = 0;
            return 0;
        }

        // PD + static friction feedforward
        double output = kP * headingError
                - kD * angularVelocity
                + kS * Math.signum(headingError);

        turnPower = Range.clip(output, -MAX_TURN_POWER, MAX_TURN_POWER);

        return turnPower;
    }

    private double none() {
        valid = false;
        turnPower = 0;
        return 0;
    }

    // ==========================================
    // ANGLE WRAPPING
    // ==========================================

    private double wrapAngle(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }

        while (radians <= -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }

    // ==========================================
    // GETTERS
    // ==========================================

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
