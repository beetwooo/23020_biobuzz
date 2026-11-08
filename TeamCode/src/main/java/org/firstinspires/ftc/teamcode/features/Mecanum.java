
package org.firstinspires.ftc.teamcode.features;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.pedropathing.follower.ManualDrive;
import com.pedropathing.drivetrain.DrivePowers;

public class Mecanum {

    Position position = new Position();

    public void fieldCentric(double ly, double lx, double rx){

        DrivePowers powers = ManualDrive.fieldCentric(
                -ly,
                -lx,
                -rx,
                position.follower.pose().heading()
        );
        ManualDrive.driveOrHold(position.follower, powers);
        position.follower.update();
    }
    
    public void resetPose(boolean startWasPressed, boolean bWasPressed){
        if (startWasPressed || bWasPressed) {
            Pose cornerPose = new Pose(0, 0, Math.toDegrees(0));
            position.follower.setPose(cornerPose);
        }
    }
}
