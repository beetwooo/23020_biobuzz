package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.features.MecanumAim;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

@TeleOp
public class TeleOpMain extends OpMode {
    private Follower follower;
    MecanumAim mecanumAim = new MecanumAim();


    @Override
    public void init(){
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void start(){

    }
    @Override
    public void loop() {

        double ly = gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        boolean startWasPressed = gamepad1.startWasPressed;
        boolean  bWasPressed = gamepad1.bWasPressed;

        mecanumAim.fieldCentric(ly, lx, rx);
        mecanum.resetPose(startWasPressed, bWasPressed);

        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

    }
    @Override
    public void stop(){

    }
}
