package org.firstinspires.ftc.teamcode.features.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Mouth {

    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING
    }

    private DcMotor LeftMotor, RightMotor;
    private Servo MouthLid;
    private State state = State.IDLE;

    public void init(HardwareMap hwMap) {

        
        LeftMotor = hwMap.get(DcMotor.class, "LM");
        RightMotor = hwMap.get(DcMotor.class, "RM");
        MouthLid = hwMap.get(Servo.class, "ML");

        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftMotor.setPower(0);
        RightMotor.setPower(0);
        MouthLid.setPosition(0.5);

        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setState(State newState) {
        state = newState;
    }

    public State getState() {
        return state;
    }

    public void update() {
        switch (state) {
            case INTAKING:
                action(0.7, 0.3, 0.5);
                break;
            case OUTTAKING:
                action(1, 1, 0.2);
                break;
            case IDLE:
            default:
                action(0, 0, 0.5);
                break;
        }
    }

    private void action(double frontPower, double backPower, double lidPosition) {
        LeftMotor.setPower(frontPower);
        RightMotor.setPower(backPower);
        MouthLid.setPosition(lidPosition);
    }

}