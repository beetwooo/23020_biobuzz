/*
package org.firstinspires.ftc.teamcode.teleOp.calibrations;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "turret pid test", group = "Calibration")
public class testtrack extends OpMode {

    private final TelemetryManager panelsTelemetry =
            PanelsTelemetry.INSTANCE.getTelemetry();

    private final Targeting turret = new Targeting();

    // =====================================
    // PANELS CONFIGURATION
    // =====================================

    public static boolean enabled = false;

    public static double targetAngleDegrees = 0;

    public static double p = 0.55;
    public static double i = 0.0;
    public static double d = 0.06;
    public static double f = 0.03;

    public static double maxPower = 0.25;

    public static double deadbandDegrees = 1.5;

    public static double minAngleDegrees = -150;
    public static double maxAngleDegrees = 150;

    public static double integralLimit = 0.5;

    public static double externalGearRatio = 1.0;

    public static boolean motorReversed = false;

    // =====================================
    // GAMEPAD EDGE DETECTION
    // =====================================

    private boolean previousA;
    private boolean previousX;
    private boolean previousY;

    private boolean previousLeft;
    private boolean previousRight;
    private boolean previousUp;
    private boolean previousDown;

    @Override
    public void init() {

        applyPanelValues();

        // Direction is applied during motor initialization.
        Targeting.MOTOR_REVERSED = motorReversed;

        turret.init(hardwareMap);

        enabled = false;

        targetAngleDegrees = 0;
    }

    @Override
    public void loop() {

        applyPanelValues();

        // =====================================
        // ENABLE / DISABLE PID
        // =====================================

        if (gamepad1.a && !previousA) {

            enabled = !enabled;
        }

        // =====================================
        // TARGET ANGLE CONTROLS
        // =====================================

        if (gamepad1.dpad_right && !previousRight) {

            targetAngleDegrees += 5;
        }

        if (gamepad1.dpad_left && !previousLeft) {

            targetAngleDegrees -= 5;
        }

        if (gamepad1.dpad_up && !previousUp) {

            targetAngleDegrees += 1;
        }

        if (gamepad1.dpad_down && !previousDown) {

            targetAngleDegrees -= 1;
        }

        // =====================================
        // RESET TARGET TO ZERO
        // =====================================

        if (gamepad1.y && !previousY) {

            targetAngleDegrees = 0;
        }

        // =====================================
        // RESET ENCODER ZERO
        // =====================================

        if (gamepad1.x && !previousX) {

            // ONLY when turret physically faces forward.
            turret.resetAngleTracking();

            targetAngleDegrees = 0;
        }

        // =====================================
        // PID CONTROL
        // =====================================

        if (enabled) {

            turret.trackAngle(
                    Math.toRadians(targetAngleDegrees)
            );

        } else {

            turret.stop();
        }

        // =====================================
        // SAVE BUTTON STATES
        // =====================================

        previousA = gamepad1.a;
        previousX = gamepad1.x;
        previousY = gamepad1.y;

        previousLeft = gamepad1.dpad_left;
        previousRight = gamepad1.dpad_right;
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;

        // =====================================
        // TELEMETRY
        // =====================================

        panelsTelemetry.addData("Enabled", enabled);

        panelsTelemetry.addData(
                "Target Angle",
                targetAngleDegrees
        );

        panelsTelemetry.addData(
                "Current Angle",
                turret.getCurrentAngleDegrees()
        );

        panelsTelemetry.addData(
                "Angle Error",
                turret.getAngleErrorDegrees()
        );

        panelsTelemetry.addData(
                "Motor Power",
                turret.getMotorPower()
        );

        panelsTelemetry.addData(
                "Encoder Ticks",
                turret.getEncoderTicks()
        );

        panelsTelemetry.addData(
                "Encoder Velocity",
                turret.getEncoderVelocity()
        );

        panelsTelemetry.addData(
                "Ticks Per Turret Revolution",
                Targeting.OUTPUT_TICKS_PER_REV
                        * Targeting.EXTERNAL_TURRET_RATIO
        );

        panelsTelemetry.addData(
                "Target Within Limits",
                turret.isTargetWithinLimits()
        );

        panelsTelemetry.addData("P", p);
        panelsTelemetry.addData("I", i);
        panelsTelemetry.addData("D", d);
        panelsTelemetry.addData("F", f);

        panelsTelemetry.addData("Max Power", maxPower);

        panelsTelemetry.addData(
                "Deadband Degrees",
                deadbandDegrees
        );

        panelsTelemetry.addData(
                "External Gear Ratio",
                externalGearRatio
        );

        panelsTelemetry.update(telemetry);
    }

    // =====================================
    // APPLY PANEL VALUES TO CONTROLLER
    // =====================================

    private void applyPanelValues() {

        Targeting.kP = Math.max(0, p);

        Targeting.kI = Math.max(0, i);

        Targeting.kD = Math.max(0, d);

        Targeting.kF = Math.max(0, f);

        Targeting.MAX_POWER =
                Math.max(0, Math.min(1.0, maxPower));

        Targeting.DEAD_BAND_DEGREES =
                Math.abs(deadbandDegrees);

        Targeting.MIN_ANGLE_DEGREES =
                Math.min(minAngleDegrees, maxAngleDegrees);

        Targeting.MAX_ANGLE_DEGREES =
                Math.max(minAngleDegrees, maxAngleDegrees);

        Targeting.EXTERNAL_TURRET_RATIO =
                Math.max(0.001, Math.abs(externalGearRatio));

        Targeting.INTEGRAL_LIMIT =
                Math.abs(integralLimit);
    }

    @Override
    public void stop() {

        enabled = false;

        turret.stop();
    }
}

 */