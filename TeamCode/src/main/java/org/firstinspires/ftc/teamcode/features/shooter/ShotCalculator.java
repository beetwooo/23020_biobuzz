
package org.firstinspires.ftc.teamcode.features.shooter;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShotCalculator {

    // =====================================
    // SHOOTING CALIBRATION POINT A
    // =====================================

    public static double POINT_A_DISTANCE = 24;

    // Encoder ticks per second.
    // Replace 0 after testing your first successful shot.
    public static double POINT_A_VELOCITY = 0;

    // =====================================
    // SHOOTING CALIBRATION POINT B
    // =====================================

    public static double POINT_B_DISTANCE = 72;

    // Encoder ticks per second.
    // Replace 0 after testing your second successful shot.
    public static double POINT_B_VELOCITY = 0;

    // =====================================
    // CALCULATE DISTANCE
    // =====================================

    public static double calculateDistance(
            double robotX,
            double robotY,
            double goalX,
            double goalY) {

        return Math.hypot(
                goalX - robotX,
                goalY - robotY
        );
    }

    // =====================================
    // CALCULATE TRAJECTORY
    // =====================================

    public static ShotSolution calculate(
            double robotX,
            double robotY,
            double goalX,
            double goalY) {

        double distance = calculateDistance(
                robotX,
                robotY,
                goalX,
                goalY
        );

        // Calibration must be valid.
        if (POINT_A_DISTANCE < 0
                || POINT_B_DISTANCE <= POINT_A_DISTANCE
                || POINT_A_VELOCITY <= 0
                || POINT_B_VELOCITY <= 0
                || !Double.isFinite(distance)
                || distance < 0.001) {

            return new ShotSolution(distance, 0, false);
        }

        // =================================
        // LINEAR INTERPOLATION
        // =================================

        double ratio =
                (distance - POINT_A_DISTANCE)
                        / (POINT_B_DISTANCE - POINT_A_DISTANCE);

        // Do not extrapolate beyond calibrated positions.
        ratio = Math.max(0, Math.min(1, ratio));

        double targetVelocity =
                POINT_A_VELOCITY
                        + ratio * (
                        POINT_B_VELOCITY
                                - POINT_A_VELOCITY
                );

        // =================================
        // MOTOR VELOCITY LIMIT
        // =================================

        double maxVelocity =
                org.firstinspires.ftc.teamcode.constants.ShooterConstants.FLYWHEEL_MAX_RPM
                        / 60.0
                        * org.firstinspires.ftc.teamcode.constants.ShooterConstants.FLYWHEEL_TPR;

        targetVelocity = Math.max(
                0,
                Math.min(targetVelocity, maxVelocity)
        );

        return new ShotSolution(
                distance,
                targetVelocity,
                true
        );
    }
}
