package org.firstinspires.ftc.teamcode.features.shooter;

import static com.bylazar.panels.Panels.stop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.constants.Field;
import org.firstinspires.ftc.teamcode.teleOp.TeleOpMain;

public class test1 {

    private DcMotorEx TM;
    Field field = new Field();
    Follower follower;
    Pose robotPose;


    private PIDFCoefficients pidfCoefficients;


    public void init(HardwareMap hardwareMap){
        TM = hardwareMap.get(DcMotorEx.class, "TM");
        TM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        TM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public test1(Follower follower){
        this.follower = follower;
    }


    public void information(){
        double x = robotPose.x();
        double y = robotPose.y();
        double heading = robotPose.heading();

        goalDistance(x, y);
        alpha(x, y);
        alphaPrime(x, y);
        turretDegrees(x, y, heading);
    }


    public void trackPoint(Pose robotPose, double goalX, double goalY) {
        alphaPrime(goalX, goalY);
        goalDistance(goalX, goalY);
        alpha(goalX, goalY);
    }

    public void trackPoint(Pose robotPose, double goalX, double goalY, double shotOffsetRadians) {
    }

    public double goalDistance(double goalX, double goalY){

        double deltaX = robotPose.x() - goalX;
        double deltaY = robotPose.y() - goalY;

        double distance = Math.sqrt(Math.pow(deltaX, 2)+ Math.pow(deltaY, 2));

        return distance;
    }

    public double alpha(double goalX, double goalY){

        double deltaX = robotPose.x() - goalX;
        double deltaY = robotPose.y() - goalY;

        double angle = Math.atan2(deltaY, deltaX);

        return Math.toDegrees(angle);
    }

    public double alphaPrime(double x, double y){
        double angle = 90 - alpha(x, y);
        return Math.toDegrees(angle);
    }

    public double turretDegrees(double x, double y, double heading){
        return alphaPrime(x, y) - heading;
    }

}
