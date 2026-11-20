
package org.firstinspires.ftc.teamcode.teleOp.calibrations;

public class testshoot {

    // =====================================
    // CALIBRATION POINTS
    // =====================================

    // Replace these velocities with actual successful
    // shooting measurements from Shooting.java.

    public static double POINT_A_DISTANCE = 24;
    public static double POINT_A_VELOCITY = 0;

    public static double POINT_B_DISTANCE = 72;
    public static double POINT_B_VELOCITY = 0;

    // =====================================
    // RESULT
    // =====================================

    public static class ShotSolution {

        public final double distance;
        public final double targetVelocity;
        public final boolean calibrated;

        public ShotSolution(
                double distance,
                double targetVelocity,
                boolean calibrated) {

            this.distance = distance;
            this.targetVelocity = targetVelocity;
            this.calibrated = calibrated;
        }
    }

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
    // CALCULATE TARGET VELOCITY
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

        // Check calibration data.
        if (POINT_A_DISTANCE == POINT_B_DISTANCE
                || POINT_A_VELOCITY <= 0
                || POINT_B_VELOCITY <= 0) {

            return new ShotSolution(
                    distance,
                    0,
                    false
            );
        }

        // Linear interpolation between two measured points.
        double ratio =
                (distance - POINT_A_DISTANCE)
                        / (POINT_B_DISTANCE - POINT_A_DISTANCE);

        // Prevent extrapolation outside calibrated distances.
        ratio = Math.max(0, Math.min(1, ratio));

        double targetVelocity =
                POINT_A_VELOCITY
                        + ratio * (
                        POINT_B_VELOCITY
                                - POINT_A_VELOCITY
                );

        double maxVelocity =
                org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_MAX_RPM
                        / 60.0
                        * org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_TPR;

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
