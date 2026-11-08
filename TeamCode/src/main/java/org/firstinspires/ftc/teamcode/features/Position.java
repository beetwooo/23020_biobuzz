package org.firstinspires.ftc.teamcode.features;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

public class Position {
    public Follower follower;

    public void update(){
        follower.update();
    }
    public void robotPose(){
        Pose robotPose = follower.pose();
    }
}
