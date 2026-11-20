
package org.firstinspires.ftc.teamcode.features.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Targeting {

    private DcMotorEx TM;

    // 5203 312 RPM encoder resolution at gearbox output
    public static double OUTPUT_TICKS_PER_REV = 537.7;

    // Additional external motor-to-turret gear ratio.
    // 1.0 means direct drive.
    public static double EXTERNAL_TURRET_RATIO = 1.0;

    // Change to REVERSE if positive motor power produces
    // the wrong physical rotation.
    public static boolean MOTOR_REVERSED = false;

    // PID gains (controller operates in radians).
    public static double kP = 0.55;
    public static double kI = 0.0;
    public static double kD = 0.06;
    public static double kF = 0.03;

    public static double MAX_POWER = 0.25;

    public static double DEAD_BAND_DEGREES = 1.5;

    public static double MIN_ANGLE_DEGREES = -150;
    public static double MAX_ANGLE_DEGREES = 150;

    public static double INTEGRAL_LIMIT = 0.5;
    public static double MAX_PID_DT_SECONDS = 0.1;

    private double targetAngleRadians = 0;

    private boolean targetWithinLimits = true;

    private double integral = 0;

    private double motorPower = 0;

    private final ElapsedTime pidTimer = new ElapsedTime();

    public void init(HardwareMap hardwareMap) {

        TM = hardwareMap.get(DcMotorEx.class, "TM");

        TM.setDirection(
                MOTOR_REVERSED
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD
        );

        TM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TM.setPower(0);

        // The turret MUST physically be facing forward
        // when this encoder reset happens.
        TM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // We use our own position PID controller.
        TM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetPID();
    }

    public double goalDistance(
            Pose robotPose,
            double goalX,
            double goalY) {

        double deltaX = goalX - robotPose.x();
        double deltaY = goalY - robotPose.y();

        return Math.hypot(deltaX, deltaY);
    }


    public double alpha(
            Pose robotPose,
            double goalX,
            double goalY) {

        double deltaX = goalX - robotPose.x();
        double deltaY = goalY - robotPose.y();

        return Math.atan2(deltaY, deltaX);
    }

    public double calculateTurretAngle(
            Pose robotPose,
            double goalX,
            double goalY,
            double shotOffsetRadians) {

        double fieldAngle = alpha(robotPose, goalX, goalY);

        double robotHeading = robotPose.heading();

        double turretAngle =
                fieldAngle
                        - robotHeading
                        - shotOffsetRadians;

        return wrapAngle(turretAngle);
    }


    public void trackPoint(
            Pose robotPose,
            double goalX,
            double goalY) {

        trackPoint(robotPose, goalX, goalY, 0);
    }


    public void trackPoint(
            Pose robotPose,
            double goalX,
            double goalY,
            double shotOffsetRadians) {

        if (robotPose == null || TM == null) {
            stop();
            return;
        }

        double target = calculateTurretAngle(
                robotPose,
                goalX,
                goalY,
                shotOffsetRadians
        );

        trackAngle(target);
    }


    public void trackAngle(double angleRadians) {

        if (TM == null) {
            return;
        }

        double requestedDegrees =
                Math.toDegrees(wrapAngle(angleRadians));

        targetWithinLimits =
                requestedDegrees >= MIN_ANGLE_DEGREES
                        && requestedDegrees <= MAX_ANGLE_DEGREES;

        double safeDegrees = Range.clip(
                requestedDegrees,
                MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES
        );

        targetAngleRadians = Math.toRadians(safeDegrees);

        updatePID();
    }



    private void updatePID() {

        double currentAngle = getCurrentAngleRadians();

        double error = targetAngleRadians - currentAngle;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        double deadband = Math.toRadians(DEAD_BAND_DEGREES);

        if (Math.abs(error) <= deadband) {

            integral = 0;
            motorPower = 0;

            TM.setPower(0);

            return;
        }

        double candidateIntegral = integral;

        if (dt > 0 && dt <= MAX_PID_DT_SECONDS) {

            candidateIntegral += error * dt;

            candidateIntegral = Range.clip(
                    candidateIntegral,
                    -INTEGRAL_LIMIT,
                    INTEGRAL_LIMIT
            );
        }

        double currentVelocityRadians =
                TM.getVelocity()
                        / getTicksPerTurretRevolution()
                        * (2.0 * Math.PI);

        double pTerm = kP * error;

        double iTerm = kI * candidateIntegral;

        double dTerm = -kD * currentVelocityRadians;

        double fTerm = kF * Math.signum(error);

        double output = pTerm + iTerm + dTerm + fTerm;

        double clippedOutput = Range.clip(
                output,
                -MAX_POWER,
                MAX_POWER
        );

        // Anti-windup.
        if (Math.abs(output) <= MAX_POWER
                || Math.signum(error) != Math.signum(output)) {

            integral = candidateIntegral;

        } else {

            output = pTerm + kI * integral + dTerm + fTerm;

            clippedOutput = Range.clip(
                    output,
                    -MAX_POWER,
                    MAX_POWER
            );
        }

        // Prevent driving farther beyond the mechanical limits.
        double currentDegrees = Math.toDegrees(currentAngle);

        if (currentDegrees >= MAX_ANGLE_DEGREES
                && clippedOutput > 0) {

            clippedOutput = 0;

        } else if (currentDegrees <= MIN_ANGLE_DEGREES
                && clippedOutput < 0) {

            clippedOutput = 0;
        }

        motorPower = clippedOutput;

        TM.setPower(motorPower);
    }

    // ==========================================
    // ENCODER CONVERSION
    // ==========================================

    private double getTicksPerTurretRevolution() {

        return OUTPUT_TICKS_PER_REV * EXTERNAL_TURRET_RATIO;
    }

    public double getCurrentAngleRadians() {

        if (TM == null) {
            return 0;
        }

        return TM.getCurrentPosition()
                / getTicksPerTurretRevolution()
                * (2.0 * Math.PI);
    }

    public double getCurrentAngleDegrees() {

        return Math.toDegrees(getCurrentAngleRadians());
    }


    public void resetAngleTracking() {

        if (TM == null) {
            return;
        }

        TM.setPower(0);

        TM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetAngleRadians = 0;

        targetWithinLimits = true;

        motorPower = 0;

        resetPID();
    }

    private void resetPID() {

        integral = 0;

        pidTimer.reset();
    }


    //angle wrapping

    private double wrapAngle(double radians) {

        while (radians > Math.PI) {
            radians -= 2.0 * Math.PI;
        }

        while (radians <= -Math.PI) {
            radians += 2.0 * Math.PI;
        }

        return radians;
    }


    //telemetry


    public double getTargetAngleDegrees() {

        return Math.toDegrees(targetAngleRadians);
    }

    public double getAngleErrorDegrees() {

        return getTargetAngleDegrees() - getCurrentAngleDegrees();
    }

    public double getMotorPower() {

        return motorPower;
    }

    public int getEncoderTicks() {

        return TM == null ? 0 : TM.getCurrentPosition();
    }

    public double getEncoderVelocity() {

        return TM == null ? 0 : TM.getVelocity();
    }

    public boolean isTargetWithinLimits() {

        return targetWithinLimits;
    }

    public boolean isBusy() {

        return Math.abs(getAngleErrorDegrees()) > DEAD_BAND_DEGREES;
    }

    public void stop() {

        if (TM != null) {

            motorPower = 0;

            TM.setPower(0);

            resetPID();
        }
    }
}