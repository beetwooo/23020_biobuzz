
package org.firstinspires.ftc.teamcode.features;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.drivetrain.DrivePowers;

public class Mecanum {

    private Follower follower;

    public Mecanum(Follower follower) {
        this.follower = follower;
    }

    public void fieldCentric(double ly, double lx, double rx) {

        DrivePowers powers = ManualDrive.fieldCentric(
                -ly,
                -lx,
                -rx,
                follower.pose().heading()
        );

        ManualDrive.driveOrHold(follower, powers);
    }

    public void resetPose(boolean startWasPressed, boolean bWasPressed) {

        if (startWasPressed || bWasPressed) {
            Pose cornerPose = new Pose(0, 0, 0);
            follower.setPose(cornerPose);
        }
    }
}
