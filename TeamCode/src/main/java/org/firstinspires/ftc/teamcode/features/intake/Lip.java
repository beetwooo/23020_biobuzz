package org.firstinspires.ftc.teamcode.features.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleOp.TeleOpMain;

public class Lip {

    private Servo LeftLip, RightLip;
    TeleOpMain teleOpMain = new TeleOpMain();

    public enum state{
        UP,
        DOWN
    }
    public void init(HardwareMap hardwareMap){
        LeftLip = hardwareMap.get(Servo.class, "LL");
        RightLip = hardwareMap.get(Servo.class, "LR");

        LeftLip.setDirection(Servo.Direction.FORWARD);
        RightLip.setDirection(Servo.Direction.REVERSE);

        LeftLip.setPosition();
        RightLip.setPosition();
    }

    public void action(boolean bWasPressed){


        if(teleOpMain.gamepad1.y){
            LeftLip.setPosition(bWasPressed);
            RightLip.setPosition(-bWasPressed);
        }
    }




    /*
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
     */

}
