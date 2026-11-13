package org.firstinspires.ftc.teamcode.features.shooter;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.Shooter;


public class Targeting {
    private CRServo leftServo;
    private CRServo rightServo;
    private AnalogInput encoder;

    private static boolean angleTrackingInitialized;
    private static double previousServoAngle;
    private static double continuousServoAngle;

    //pid
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double integral;
    private double previousError;

    private double targetAngleRadians;
    private double currentAngleRadians;
    private double servoPower;


    public void init(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(
                CRServo.class, Shooter.TURRET_LEFT_SERVO_NAME);
        rightServo = hardwareMap.get(
                CRServo.class, Shooter.TURRET_RIGHT_SERVO_NAME);
        encoder = hardwareMap.get(
                AnalogInput.class, Shooter.TURRET_ENCODER_NAME);

        double servoAngle = readServoAngle();
        if (!angleTrackingInitialized) {
            // 새로 시작할 때는 케이블을 풀고 터렛을 정면에 놓아야 한다.
            // 단회전 엔코더만으로는 이전 전원의 누적 회전수를 복원할 수 없다.
            previousServoAngle = servoAngle;
            continuousServoAngle = 0;
            angleTrackingInitialized = true;
        } else {
            // Red/Blue OpMode 전환 중 발생한 작은 변화도 기존 누적각에 이어 붙인다.
            continuousServoAngle += wrapAngle(servoAngle - previousServoAngle);
            previousServoAngle = servoAngle;
        }
        updateCurrentAngle();

        resetPid();
        setServoPower(0);
    }


    public void trackPoint(Pose robotPose, double goalX, double goalY) {
        trackPoint(robotPose, goalX, goalY, 0);
    }

    public void trackPoint(Pose robotPose, double goalX, double goalY,
                           double shotOffsetRadians) {
        if (robotPose == null) {
            stop();
            return;
        }

        targetAngleRadians = calculateTargetAngle(
                robotPose, goalX, goalY, shotOffsetRadians);
        trackAngle(targetAngleRadians);
    }

    public void trackAngle(double angleRadians) {
        targetAngleRadians = Range.clip(
                wrapAngle(angleRadians),
                Shooter.TURRET_MIN_ANGLE_RADIANS,
                Shooter.TURRET_MAX_ANGLE_RADIANS);
        updateCurrentAngle();

        //if 180+ -> -180 then it turns the opposite
        double error = targetAngleRadians - currentAngleRadians;
        double power = calculatePidPower(error);

        if ((currentAngleRadians >= Shooter.TURRET_MAX_ANGLE_RADIANS && power > 0)
                || (currentAngleRadians <= Shooter.TURRET_MIN_ANGLE_RADIANS && power < 0)) {
            power = 0;
        }
        setServoPower(power);
    }

    public void resetAngleTracking() {
        previousServoAngle = readServoAngle();
        continuousServoAngle = 0;
        updateCurrentAngle();
        resetPid();
    }

    public void stop() {
        resetPid();
        if (leftServo != null && rightServo != null) {
            setServoPower(0);
        }
    }

    
    
    private double calculateTargetAngle(Pose robotPose, double goalX, double goalY,
                                        double shotOffsetRadians) {
        double robotX = robotPose.x();
        double robotY = robotPose.y();
        double robotHeading = robotPose.heading();
        double fieldAngleToGoal = Math.atan2(goalY - robotY, goalX - robotX);

        
        
        return wrapAngle(fieldAngleToGoal - robotHeading - shotOffsetRadians);
    }

    private void updateCurrentAngle() {
        double servoAngle = readServoAngle();
        double angleChange = wrapAngle(servoAngle - previousServoAngle);

        continuousServoAngle += angleChange;
        previousServoAngle = servoAngle;

        // 기어비: 서보 회전수 / 터렛 회전수
        currentAngleRadians = continuousServoAngle / Shooter.TURRET_GEAR_RATIO;
    }


    private double readServoAngle() {
        double angle = encoder.getVoltage()
                / encoder.getMaxVoltage() * 2.0 * Math.PI;

        return wrapAngle(angle * Shooter.TURRET_ENCODER_DIRECTION);
    }

    private double calculatePidPower(double error) {
        double dt = pidTimer.seconds();
        pidTimer.reset();

        double derivative = 0;
        if (dt > 0 && dt <= Shooter.TURRET_MAX_PID_DT_SECONDS) {
            integral = Range.clip(
                    integral + error * dt,
                    -Shooter.TURRET_INTEGRAL_LIMIT,
                    Shooter.TURRET_INTEGRAL_LIMIT);
            derivative = (error - previousError) / dt;
        }
        previousError = error;

        // 목표 각도 근처에서는 떨림을 막기 위해 서보를 정지한다.
        if (Math.abs(error) <= Shooter.TURRET_DEADBAND_RADIANS) {
            integral = 0;
            return 0;
        }

        double power = Shooter.TURRET_P * error
                + Shooter.TURRET_I * integral
                + Shooter.TURRET_D * derivative
                + Shooter.TURRET_F * Math.signum(error);

        return Range.clip(
                power,
                -Shooter.TURRET_MAX_POWER,
                Shooter.TURRET_MAX_POWER);
    }

    private void setServoPower(double power) {
        servoPower = power;
        leftServo.setPower(power * Shooter.TURRET_LEFT_POWER_DIRECTION);
        rightServo.setPower(power * Shooter.TURRET_RIGHT_POWER_DIRECTION);
    }

    private void resetPid() {
        integral = 0;
        previousError = 0;
        pidTimer.reset();
    }

    private double wrapAngle(double radians) {
        while (radians > Math.PI) {
            radians -= 2.0 * Math.PI;
        }
        while (radians <= -Math.PI) {
            radians += 2.0 * Math.PI;
        }
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
        return encoder.getVoltage();
    }

    public double getServoPower() {
        return servoPower;
    }
}
