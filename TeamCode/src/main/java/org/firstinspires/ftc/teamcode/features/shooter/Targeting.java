package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Turret controller for the FTC robot.
 *
 * The analog encoder measures one servo-shaft revolution. The controller
 * unwraps each reading and accumulates the servo-shaft angle so multiple
 * revolutions can be tracked while the Robot Controller remains powered.
 */
public class Targeting {

    // ---------------------------------------------------------------------
    // Panels-tunable turret constants. Tracking.java copies its dashboard
    // values into these fields while the calibration OpMode is running.
    // ---------------------------------------------------------------------

    public static double turret_P = 0.364;
    public static double turret_I = 0.0;
    public static double turret_D = 0.001;
    public static double turret_F = 0.0017;

    public static double TURRET_DEADBAND_RADIANS = Math.toRadians(2.0);
    public static double TURRET_MAX_POWER = 0.7;
    public static double TURRET_INTEGRAL_LIMIT = 0.5;
    public static double TURRET_MAX_PID_DT_SECONDS = 0.1;

    // Mechanical travel limits. Set these to the real BioBuzz turret limits.
    public static double TURRET_MIN_ANGLE_RADIANS = Math.toRadians(-180.0);
    public static double TURRET_MAX_ANGLE_RADIANS = Math.toRadians(180.0);

    // Hardware names. These match the previous-season project; verify your
    // BioBuzz Robot Configuration uses the same names.
    public static String TURRET_LEFT_SERVO_NAME = "TL";
    public static String TURRET_RIGHT_SERVO_NAME = "TR";
    public static String TURRET_ENCODER_NAME = "Tencoder";

    // IMPORTANT: convention is servo rotations / one turret rotation.
    // 2.1 is the DECODE ratio, NOT a confirmed BioBuzz ratio. Replace it
    // with the measured BioBuzz ratio before enabling the turret.
    public static double TURRET_GEAR_RATIO = 2.1;

    // +1 means the encoder angle increases for counterclockwise turret motion.
    public static double TURRET_ENCODER_DIRECTION = 1.0;

    // Multipliers that make both servos rotate the turret in the same direction.
    public static double TURRET_LEFT_POWER_DIRECTION = -1.0;
    public static double TURRET_RIGHT_POWER_DIRECTION = -1.0;

    // ---------------------------------------------------------------------
    // Hardware and controller state
    // ---------------------------------------------------------------------

    private CRServo leftServo;
    private CRServo rightServo;
    private AnalogInput encoder;
    private boolean initialized = false;

    // Static because OpModes may transition without the Robot Controller
    // process restarting. These values cannot survive a full power reset.
    private static boolean angleTrackingInitialized = false;
    private static double previousServoAngle = 0.0;
    private static double continuousServoAngle = 0.0;

    private final ElapsedTime pidTimer = new ElapsedTime();
    private double integral = 0.0;
    private double previousError = 0.0;

    private double targetAngleRadians = 0.0;
    private double currentAngleRadians = 0.0;
    private double servoPower = 0.0;

    public void init(HardwareMap hardwareMap) {
        validateConfiguration();

        leftServo = hardwareMap.get(CRServo.class, TURRET_LEFT_SERVO_NAME);
        rightServo = hardwareMap.get(CRServo.class, TURRET_RIGHT_SERVO_NAME);
        encoder = hardwareMap.get(AnalogInput.class, TURRET_ENCODER_NAME);
        initialized = true;

        double servoAngle = readServoAngle();
        if (!angleTrackingInitialized) {
            // The first reading establishes the reference position as 0 deg.
            previousServoAngle = servoAngle;
            continuousServoAngle = 0.0;
            angleTrackingInitialized = true;
        } else {
            // Continue tracking small movement between OpMode transitions.
            continuousServoAngle += wrapAngle(servoAngle - previousServoAngle);
            previousServoAngle = servoAngle;
        }

        updateCurrentAngle();
        resetPid();
        setServoPower(0.0);
    }

    /** Track a fixed turret angle relative to the robot. */
    public void trackAngle(double angleRadians) {
        if (!initialized) return;

        targetAngleRadians = Range.clip(
                wrapAngle(angleRadians),
                TURRET_MIN_ANGLE_RADIANS,
                TURRET_MAX_ANGLE_RADIANS
        );

        updateCurrentAngle();

        // Do not wrap this error: currentAngleRadians is an accumulated angle.
        double error = targetAngleRadians - currentAngleRadians;
        double power = calculatePidPower(error);

        // Prevent driving farther past either mechanical limit.
        if ((currentAngleRadians >= TURRET_MAX_ANGLE_RADIANS && power > 0.0)
                || (currentAngleRadians <= TURRET_MIN_ANGLE_RADIANS && power < 0.0)) {
            power = 0.0;
        }

        setServoPower(power);
    }

    /**
     * Track a field position using Pedro Pathing 3's Pose.
     * Pose headings are radians; goalX and goalY use the same field units
     * as the robot pose (normally inches).
     */
    public void trackPoint(Pose robotPose, double goalX, double goalY) {
        trackPoint(robotPose, goalX, goalY, 0.0);
    }

    /** Track a field position with an optional shot/motion compensation angle. */
    public void trackPoint(
            Pose robotPose,
            double goalX,
            double goalY,
            double shotOffsetRadians
    ) {
        if (!initialized || robotPose == null) {
            stop();
            return;
        }

        double fieldAngleToGoal = Math.atan2(
                goalY - robotPose.y(),
                goalX - robotPose.x()
        );

        double robotRelativeAngle = wrapAngle(
                fieldAngleToGoal - robotPose.heading() - shotOffsetRadians
        );

        trackAngle(robotRelativeAngle);
    }

    /** Set the current physical turret position as 0 degrees. */
    public void resetAngleTracking() {
        if (!initialized) return;

        previousServoAngle = readServoAngle();
        continuousServoAngle = 0.0;
        targetAngleRadians = 0.0;
        updateCurrentAngle();
        resetPid();
    }

    /** Reset PID history without changing the measured turret position. */
    public void resetController() {
        resetPid();
    }

    public void stop() {
        resetPid();
        if (initialized) {
            setServoPower(0.0);
        }
    }

    private void updateCurrentAngle() {
        if (!initialized) return;

        double servoAngle = readServoAngle();
        double angleChange = wrapAngle(servoAngle - previousServoAngle);
        continuousServoAngle += angleChange;
        previousServoAngle = servoAngle;

        // Gear ratio convention: servo rotations / turret rotation.
        currentAngleRadians = continuousServoAngle / TURRET_GEAR_RATIO;
    }

    private double readServoAngle() {
        double maxVoltage = encoder.getMaxVoltage();
        if (maxVoltage <= 0.0) return 0.0;

        double angle = (encoder.getVoltage() / maxVoltage) * 2.0 * Math.PI;
        return wrapAngle(angle * TURRET_ENCODER_DIRECTION);
    }

    private double calculatePidPower(double error) {
        double dt = pidTimer.seconds();
        pidTimer.reset();

        double derivative = 0.0;
        if (dt > 0.0 && dt <= TURRET_MAX_PID_DT_SECONDS) {
            integral = Range.clip(
                    integral + error * dt,
                    -TURRET_INTEGRAL_LIMIT,
                    TURRET_INTEGRAL_LIMIT
            );
            derivative = (error - previousError) / dt;
        }
        previousError = error;

        if (Math.abs(error) <= TURRET_DEADBAND_RADIANS) {
            integral = 0.0;
            return 0.0;
        }

        double power = turret_P * error
                + turret_I * integral
                + turret_D * derivative
                + turret_F * Math.signum(error);

        return Range.clip(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);
    }

    private void setServoPower(double power) {
        double limitedPower = Range.clip(power, -1.0, 1.0);
        servoPower = limitedPower;

        leftServo.setPower(limitedPower * TURRET_LEFT_POWER_DIRECTION);
        rightServo.setPower(limitedPower * TURRET_RIGHT_POWER_DIRECTION);
    }

    private void resetPid() {
        integral = 0.0;
        previousError = 0.0;
        pidTimer.reset();
    }

    private void validateConfiguration() {
        if (!Double.isFinite(TURRET_GEAR_RATIO) || TURRET_GEAR_RATIO <= 0.0) {
            throw new IllegalStateException("TURRET_GEAR_RATIO must be a positive number.");
        }
        if (TURRET_MAX_POWER < 0.0 || TURRET_MAX_POWER > 1.0) {
            throw new IllegalStateException("TURRET_MAX_POWER must be between 0 and 1.");
        }
        if (TURRET_MIN_ANGLE_RADIANS >= TURRET_MAX_ANGLE_RADIANS) {
            throw new IllegalStateException("Turret minimum angle must be below maximum angle.");
        }
    }

    private double wrapAngle(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians <= -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }

    public double getTargetAngleDegrees() {
        return Math.toDegrees(targetAngleRadians);
    }

    public double getCurrentAngleDegrees() {
        return Math.toDegrees(currentAngleRadians);
    }

    public double getAngleErrorDegrees() {
        return Math.toDegrees(targetAngleRadians - currentAngleRadians);
    }

    public double getEncoderVoltage() {
        return initialized ? encoder.getVoltage() : 0.0;
    }

    public double getServoPower() {
        return servoPower;
    }
}
