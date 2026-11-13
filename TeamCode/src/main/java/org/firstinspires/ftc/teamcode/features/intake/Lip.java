package org.firstinspires.ftc.teamcode.features.intake;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lip {

    private Servo LeftServo, RightServo;
    private State state = State.DOWN;

    public enum State {
        UP,
        DOWN
    }
    public void init(HardwareMap hardwareMap){
        LeftServo = hardwareMap.get(Servo.class, "LL");
        RightServo = hardwareMap.get(Servo.class, "LR");

        LeftServo.setDirection(Servo.Direction.FORWARD);
        RightServo.setDirection(Servo.Direction.REVERSE);

        LeftServo.setPosition(-1);
        RightServo.setPosition(1);
    }

    public void setState(State newState) {
        state = newState;
    }

    public State getState() {
        return state;
    }

    public void update() {
        switch (state) {
            case UP:
                action(1, -1);
                break;
            case DOWN:
                action(-1, 1);
                break;
        }
    }

    private void action(double leftPosition, double rightPosition) {
        LeftServo.setPosition(leftPosition);
        RightServo.setPosition(rightPosition);
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
