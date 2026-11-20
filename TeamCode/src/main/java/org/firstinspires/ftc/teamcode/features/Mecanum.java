
package org.firstinspires.ftc.teamcode.features;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;

public class Mecanum {

    private final Follower follower;

    public Mecanum(Follower follower) {
        this.follower = follower;
    }

    /**
     * Field-centric drive in Pedro 3's convention:
     *   forward + = robot forward, strafe + = robot left, turn + = counter-clockwise.
     * Convert the sticks before calling (forward = -left_stick_y,
     * strafe = -left_stick_x, turn = -right_stick_x).
     *
     * driverHeadingRadians = field direction the driver faces, so pushing the
     * stick forward always moves the robot away from the driver.
     *
     * follower.update() must still be called once every loop.
     */
    public void fieldCentric(double forward, double strafe, double turn, double driverHeadingRadians) {

        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                strafe,
                turn,
                follower.pose().heading(),
                -driverHeadingRadians
        );

        ManualDrive.driveOrHold(follower, powers);
    }

    /**
     * Same as fieldCentric, but always drives with these powers and never switches
     * to Pedro's HOLD. Use while auto-aiming: driveOrHold locks the robot in place
     * once the turn power drops below 0.1, which stopped the aim a few degrees short.
     */
    public void fieldCentricNoHold(double forward, double strafe, double turn, double driverHeadingRadians) {

        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                strafe,
                turn,
                follower.pose().heading(),
                -driverHeadingRadians
        );

        follower.manual(powers);
    }

    /** Resets the pose. Only press when the robot is physically at that pose. */
    public void resetPose(boolean resetPressed, Pose pose) {

        if (resetPressed) {
            follower.setPose(pose);
        }
    }
}
