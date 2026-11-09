package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.features.Mecanum;
import org.firstinspires.ftc.teamcode.features.Position;
import org.firstinspires.ftc.teamcode.features.intake.Lip;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TELEOP_TEST", group = "TeleOp")
public class TeleOpMain extends OpMode {
    public Follower follower;
    Position position = new Position();
    Pose robotPose = follower.pose();
    Mecanum mecanum = new Mecanum();
    Lip lip = new Lip();


    @Override
    public void init(){
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void start(){

    }
    @Override
    public void loop() {

        position.update();
        position.robotPose();

        //stick
        double ly = gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double ry = gamepad1.right_stick_y;

        boolean startWasPressed = gamepad1.startWasPressed();
        boolean backWasPressed = gamepad1.backWasPressed();

        boolean bWasPressed = gamepad1.bWasPressed();
        boolean aWasPressed = gamepad1.aWasPressed();
        boolean xWasPressed = gamepad1.xWasPressed();
        boolean yWasPressed = gamepad1.yWasPressed();

        boolean




        mecanum.fieldCentric(ly, lx, rx);

        mecanum.resetPose(startWasPressed, bWasPressed);
        lip.action(bWasPressed);

        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

    }
    @Override
    public void stop(){

    }
}
