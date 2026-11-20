package org.firstinspires.ftc.teamcode.features.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING
    }

    private DcMotor intakeMotor, backSpinMotor;
    private Servo gateServo;
    private State state = State.IDLE;

    public void init(HardwareMap hardwareMap) {

        
        intakeMotor = hardwareMap.get(DcMotor.class, "LM");
        gateServo = hardwareMap.get(Servo.class, "ML");
        backSpinMotor= hardwareMap.get(DcMotor.class, "BSM");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backSpinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setPower(0);
        backSpinMotor.setPower(0);
        gateServo.setPosition(0.5);


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

    private void action(double intakePower, double gatePosition, double backSpinPower) {
        intakeMotor.setPower(intakePower);
        gateServo.setPosition(gatePosition);
        backSpinMotor.setPower(backSpinPower);
    }

}