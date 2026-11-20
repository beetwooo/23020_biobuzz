
package org.firstinspires.ftc.teamcode.features.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Flywheel {

    public enum State {
        IDLE,
        SPINNING_UP,
        READY
    }

    private DcMotorEx flywheel;

    private State state = State.IDLE;

    private double targetVelocity = 0;

    private boolean inTolerance = false;

    private final ElapsedTime readyTimer = new ElapsedTime();

    // =====================================
    // INITIALIZATION
    // =====================================

    public void init(HardwareMap hardwareMap) {

        flywheel = hardwareMap.get(
                DcMotorEx.class,
                org.firstinspires.ftc.teamcode.constants.ShooterConstants.FLYWHEEL_NAME
        );

        flywheel.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        flywheel.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        flywheel.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        applyPIDF();

        flywheel.setPower(0);
    }

    // =====================================
    // PIDF
    // =====================================

    public void applyPIDF() {

        PIDFCoefficients coefficients =
                new PIDFCoefficients(
                        org.firstinspires.ftc.teamcode.constants.ShooterConstants.FLYWHEEL_P,
                        org.firstinspires.ftc.teamcode.constants.ShooterConstants.FLYWHEEL_I,
                        org.firstinspires.ftc.teamcode.constants.ShooterConstants.FLYWHEEL_D,
                        org.firstinspires.ftc.teamcode.constants.ShooterConstants.FLYWHEEL_F
                );

        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                coefficients
        );
    }

    // =====================================
    // SET TARGET VELOCITY
    // =====================================

    public void setTargetVelocity(double velocity) {

        double maxVelocity =
                org.firstinspires.ftc.teamcode.constants.ShooterConstants.FLYWHEEL_MAX_RPM
                        / 60.0
                        * org.firstinspires.ftc.teamcode.constants.ShooterConstants.FLYWHEEL_TPR;

        double newTarget = Range.clip(
                velocity,
                0,
                maxVelocity
        );

        if (Math.abs(newTarget - targetVelocity) > 1) {

            inTolerance = false;

            state = newTarget > 0
                    ? State.SPINNING_UP
                    : State.IDLE;
        }

        targetVelocity = newTarget;
    }

    // =====================================
    // AIM USING TRAJECTORY CALCULATOR
    // =====================================

    public ShotSolution aimAt(
            double robotX,
            double robotY,
            double goalX,
            double goalY) {

        ShotSolution trajectory =
                ShotCalculator.calculate(
                        robotX,
                        robotY,
                        goalX,
                        goalY
                );

        if (trajectory.isValid()) {

            setTargetVelocity(
                    trajectory.getTargetVelocity()
            );

        } else {

            setTargetVelocity(0);
        }

        return trajectory;
    }

    // =====================================
    // MOTOR UPDATE
    // =====================================

    public void update() {

        if (flywheel == null) {
            return;
        }

        if (targetVelocity <= 0) {

            flywheel.setPower(0);

            state = State.IDLE;

            inTolerance = false;

            return;
        }

        flywheel.setVelocity(targetVelocity);

        double error = Math.abs(
                targetVelocity - flywheel.getVelocity()
        );

        boolean withinTolerance =
                error <= org.firstinspires.ftc.teamcode.constants.ShooterConstants.FLYWHEEL_READY_TOLERANCE;

        if (withinTolerance) {

            if (!inTolerance) {

                readyTimer.reset();

                inTolerance = true;
            }

            if (readyTimer.seconds()
                    >= org.firstinspires.ftc.teamcode.constants.ShooterConstants.FLYWHEEL_READY_TIME) {

                state = State.READY;

            } else {

                state = State.SPINNING_UP;
            }

        } else {

            inTolerance = false;

            state = State.SPINNING_UP;
        }
    }

    // =====================================
    // STOP
    // =====================================

    public void stop() {

        targetVelocity = 0;

        state = State.IDLE;

        inTolerance = false;

        if (flywheel != null) {
            flywheel.setPower(0);
        }
    }

    // =====================================
    // GETTERS
    // =====================================

    public State getState() {
        return state;
    }

    public boolean isReady() {
        return state == State.READY;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocity() {

        return flywheel == null
                ? 0
                : flywheel.getVelocity();
    }

    public double getVelocityError() {
        return targetVelocity - getCurrentVelocity();
    }

    public double getMotorPower() {

        return flywheel == null
                ? 0
                : flywheel.getPower();
    }

    public int getEncoderTicks() {

        return flywheel == null
                ? 0
                : flywheel.getCurrentPosition();
    }
}
