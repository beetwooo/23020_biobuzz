package org.firstinspires.ftc.teamcode.features.shooter;

import static org.firstinspires.ftc.teamcode.constants.ShooterConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter flywheel. Everything is in RPM; settings live in ShooterConstants.
 *
 * Use:
 *   init(hardwareMap)          once
 *   spinUpForDistance(inches)  or setTargetRpm(rpm) / stop()
 *   update()                   every loop
 *   isReady()                  true when up to speed
 */
public class Flywheel {

    private DcMotorEx motor;
    private double targetRpm = 0;
    private final ElapsedTime readyTimer = new ElapsedTime();

    // Last PIDF sent to the hub, so Panels changes are picked up live.
    private double sentP, sentI, sentD, sentF;
    private PIDFCoefficients defaultPIDF;

    public void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR);
        motor.setDirection(FLYWHEEL_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Remember the hub's default PIDF so it can be used until you tune your own.
        defaultPIDF = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        sendPIDF();
        motor.setPower(0);
        targetRpm = 0;
    }

    // ===== Commands =====

    public void setTargetRpm(double rpm) {
        targetRpm = Range.clip(rpm, 0, MAX_RPM);
    }

    public void spinUpForDistance(double inches) {
        setTargetRpm(rpmForDistance(inches));
    }

    public void stop() {
        targetRpm = 0;
        if (motor != null) {
            motor.setPower(0);
        }
    }

    /** Call every loop. */
    public void update() {
        if (motor == null) {
            return;
        }

        if (P != sentP || I != sentI || D != sentD || F != sentF) {
            sendPIDF();
        }

        if (targetRpm <= 0) {
            motor.setPower(0);
        } else {
            motor.setVelocity(rpmToTicksPerSecond(targetRpm));
        }

        // Restart the ready timer whenever we are off target.
        if (targetRpm <= 0 || Math.abs(getErrorRpm()) > READY_TOLERANCE_RPM) {
            readyTimer.reset();
        }
    }

    // ===== Distance -> RPM =====

    /** True once NEAR_RPM and FAR_RPM have been filled in. */
    public static boolean isCalibrated() {
        return NEAR_RPM > 0 && FAR_RPM > 0 && FAR_DISTANCE > NEAR_DISTANCE;
    }

    /** Straight-line blend between the two calibration shots. */
    public static double rpmForDistance(double inches) {
        if (!isCalibrated()) {
            return DEFAULT_RPM;
        }
        double t = (inches - NEAR_DISTANCE) / (FAR_DISTANCE - NEAR_DISTANCE);
        t = Range.clip(t, 0, 1);
        return NEAR_RPM + t * (FAR_RPM - NEAR_RPM);
    }

    // ===== Status =====

    public boolean isReady() {
        return targetRpm > 0 && readyTimer.seconds() >= READY_TIME;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getCurrentRpm() {
        return motor == null ? 0 : ticksPerSecondToRpm(motor.getVelocity());
    }

    public double getErrorRpm() {
        return targetRpm - getCurrentRpm();
    }

    public double getPower() {
        return motor == null ? 0 : motor.getPower();
    }

    // ===== Helpers =====

    private void sendPIDF() {
        if (P == 0 && I == 0 && D == 0 && F == 0) {
            // Not tuned yet: put the hub's own default PIDF back.
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, defaultPIDF);
        } else {
            motor.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(P, I, D, F));
        }
        sentP = P;
        sentI = I;
        sentD = D;
        sentF = F;
    }

    private static double rpmToTicksPerSecond(double rpm) {
        return rpm / 60.0 * TICKS_PER_REV;
    }

    private static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond / TICKS_PER_REV * 60.0;
    }
}
