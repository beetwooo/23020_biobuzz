
package org.firstinspires.ftc.teamcode.features.shooter;

public class ShotSolution {

    private final double distance;
    private final double targetVelocity;
    private final boolean valid;

    public ShotSolution(
            double distance,
            double targetVelocity,
            boolean valid) {

        this.distance = distance;
        this.targetVelocity = targetVelocity;
        this.valid = valid;
    }

    public double getDistance() {
        return distance;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public boolean isValid() {
        return valid;
    }
}
