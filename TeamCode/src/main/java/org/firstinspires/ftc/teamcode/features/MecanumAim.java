package org.firstinspires.ftc.teamcode.features;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.datas.FieldCentric;
import org.firstinspires.ftc.teamcode.datas.Positions;

public class MecanumAim {

    Follower follower;

    double goalX, goalY;
    double kP = 1.0;

    public void init(HardwareMap hardwareMap) {
        Pose robotPose = follower.pose();

        if (robotPose.x() >= 72) {
            goalX = Positions.GOAL_RED_FRONT_X;
            goalY = Positions.GOAL_RED_FRONT_Y;
        } else {
            goalX = Positions.GOAL_RED_BACK_X;
            goalY = Positions.GOAL_RED_BACK_Y;
        }

    }

    public double getRotationPower() {
        Pose robotPose = follower.pose();

        double dx = goalX - robotPose.x();
        double dy = goalY - robotPose.y();

        double targetHeading = Math.atan2(dy, dx);
        double headingError = targetHeading - robotPose.heading();

        return headingError * kP;
    }

}


