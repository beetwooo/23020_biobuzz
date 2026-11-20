
package org.firstinspires.ftc.teamcode.features.shooter;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

public class test2 {

    // PID starting values
    public static double kP = 1.5;

    public static double MAX_TURN_POWER = 0.35;

    public static double TOLERANCE_DEGREES = 2.0;

    // Angle between robot forward and fixed shooter direction.
    // 0 means shooter faces directly forward.
    public static double SHOOTER_MOUNT_OFFSET_RADIANS = 0;

    private double targetHeading;
    private double headingError;
    private double turnPower;

    // ==========================================
    // AIM AT GOAL
    // ==========================================

    public double calculate(
            Pose robotPose,
            double goalX,
            double goalY,
            double shotOffsetRadians) {

        if (robotPose == null) {
            turnPower = 0;
            return 0;
        }

        // Robot-to-goal vector
        double deltaX = goalX - robotPose.x();
        double deltaY = goalY - robotPose.y();

        if (Math.hypot(deltaX, deltaY) < 0.001) {
            turnPower = 0;
            return 0;
        }

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
        if (Math.abs(headingError)
                <= Math.toRadians(TOLERANCE_DEGREES)) {

            turnPower = 0;
            return 0;
        }

        // Proportional controller
        turnPower = Range.clip(
                kP * headingError,
                -MAX_TURN_POWER,
                MAX_TURN_POWER
        );

        return turnPower;
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

    public boolean isAimed() {
        return Math.abs(headingError)
                <= Math.toRadians(TOLERANCE_DEGREES);
    }
}
