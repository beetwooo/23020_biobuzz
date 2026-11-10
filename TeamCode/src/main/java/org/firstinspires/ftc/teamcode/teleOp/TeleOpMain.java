package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.features.Mecanum;
import org.firstinspires.ftc.teamcode.features.Position;
import org.firstinspires.ftc.teamcode.features.intake.Lip;
import org.firstinspires.ftc.teamcode.features.intake.Mouth;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TELEOP_TEST", group = "TeleOp")
public class TeleOpMain extends OpMode {
    public Follower follower;
    private Position position = new Position();
    private Pose robotPose = follower.pose();
    private Mouth mouth = new Mouth();
    private Lip lip = new Lip();
    private Mecanum mecanum = new Mecanum();


    @Override
    public void init(){
        follower = Constants.create(hardwareMap);
        mouth.init(hardwareMap);
        lip.init(hardwareMap);

        if(gamepad1.left_bumper){
            mouth.setState(Mouth.State.INTAKING);
        }else if(gamepad1.right_bumper){
            mouth.setState(Mouth.State.OUTTAKING);
        }else{
            mouth.setState(Mouth.State.IDLE);
        }

        if(gamepad1.left_bumper){
            lip.setState(Lip.State.DOWN);
        }else {
            lip.setState(Lip.State.UP);
        }

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





        mecanum.fieldCentric(ly, lx, rx);

        mecanum.resetPose(startWasPressed, bWasPressed);

        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

    }
    @Override
    public void stop(){

    }
}
