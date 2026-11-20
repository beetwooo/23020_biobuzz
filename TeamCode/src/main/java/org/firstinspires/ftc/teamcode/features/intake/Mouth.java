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

    private DcMotor IntakingMotor, BackSpinMotor;
    private Servo Trigger;
    private State state = State.IDLE;

    public void init(HardwareMap hardwareMap) {

        
        IntakingMotor = hardwareMap.get(DcMotor.class, "LM");
        Trigger = hardwareMap.get(Servo.class, "ML");
        BackSpinMotor= hardwareMap.get(DcMotor.class, "BSM");

        IntakingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackSpinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IntakingMotor.setPower(0);
        BackSpinMotor.setPower(0);
        Trigger.setPosition(0.5);


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
                action(0.7, 0.5, 0);
                break;
            case OUTTAKING:
                action(1,  0.2, 1);
                break;
            case IDLE:
            default:
                action(0,  0.5, 0);
                break;
        }
    }

    private void action(double frontPower, double lidPosition, double BackSpinPower) {
        IntakingMotor.setPower(frontPower);
        Trigger.setPosition(lidPosition);
        BackSpinMotor.setPower(BackSpinPower);
    }

}