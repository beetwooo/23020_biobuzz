package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.features.Mecanum;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TELEOP_TEST_2", group = "TeleOp")
public class test extends OpMode {
    private Follower follower;
    Mecanum mecanum = new Mecanum();


    @Override
    public void init(){
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void start(){

    }
    @Override
    public void loop() {

        follower.update();
        Pose robotPose = follower.pose();

        //stick
        double ly = gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double ry = gamepad1.right_stick_y;

        boolean startWasPressed = gamepad1.startWasPressed();
        boolean bWasPressed = gamepad1.bWasPressed();

        DrivePowers powers = ManualDrive.fieldCentric(
                -ly,
                -lx,
                -rx,
                follower.pose().heading()
        );
        ManualDrive.driveOrHold(follower, powers);

        if (startWasPressed || bWasPressed) {
            Pose cornerPose = new Pose(0, 0, Math.toDegrees(0));
            follower.setPose(cornerPose);
        }

        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

    }
    @Override
    public void stop(){

    }
}
