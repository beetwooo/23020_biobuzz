package org.firstinspires.ftc.teamcode.main;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.features.Mecanum;

public class TeleOpMain extends OpMode {

    private Follower follower;
    private Pose pose;

    Mecanum mecanum = new Mecanum();
    private double driverHeadingDegrees;

    @Override
    public void init() {

    }

    @Override
    public void loop() {

        double ly = gamepad1.left_stick_y;
        double lx = gamepad1. left_stick_x;
        double ry = gamepad1.right_stick_y;
        double rx = gamepad1.right_stick_x;

        boolean startWasPressed = gamepad1.startWasPressed();
        boolean bWasPressed = gamepad1.bWasPressed();      // No parentheses


        //field centric + mecanum
        mecanum.fieldCentric(ly, lx, rx);
        //재위치
        mecanum.resetPose(startWasPressed, bWasPressed);

        follower.update();
        Pose robotPose = follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
    }
}
