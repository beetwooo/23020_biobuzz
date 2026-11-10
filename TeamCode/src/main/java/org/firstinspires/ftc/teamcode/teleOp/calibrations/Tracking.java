package org.firstinspires.ftc.teamcode.teleOp.calibrations;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.Turret;

/**
 * Panels-based manual turret calibration OpMode.
 *
 * Set enabled = true in Panels to move the turret toward targetAngleDegrees.
 * Set enabled = false to stop the turret.
 */
@Configurable
@TeleOp(name = "BioBuzz Turret Tracking Calibration", group = "Calibration")
public class Tracking extends OpMode {

    private final TelemetryManager panelsTelemetry =
            PanelsTelemetry.INSTANCE.getTelemetry();

    private final Turret turret = new Turret();

    // Main controls
    public static boolean enabled = false;
    public static double targetAngleDegrees = 0.0;

    // Set this true only when the turret is physically at its chosen 0-degree
    // reference position and enabled is false. It resets the accumulated angle.
    public static boolean resetAngleToZero = false;

    // PIDF values
    public static double p = Turret.turret_P;
    public static double i = Turret.turret_I;
    public static double d = Turret.turret_D;
    public static double f = Turret.turret_F;

    // Mechanical and controller settings
    public static double maxPower = Turret.TURRET_MAX_POWER;
    public static double deadbandDegrees =
            Math.toDegrees(Turret.TURRET_DEADBAND_RADIANS);
    public static double gearRatio = Turret.TURRET_GEAR_RATIO;
    public static double minAngleDegrees =
            Math.toDegrees(Turret.TURRET_MIN_ANGLE_RADIANS);
    public static double maxAngleDegrees =
            Math.toDegrees(Turret.TURRET_MAX_ANGLE_RADIANS);

    private double previousGearRatio = gearRatio;

    @Override
    public void init() {
        enabled = false;
        resetAngleToZero = false;

        applyPanelValues();
        turret.init(hardwareMap);

        // Calibration starts with the turret's current physical position as 0.
        // Put it at the reference position before initializing this OpMode.
        turret.resetAngleTracking();
        previousGearRatio = gearRatio;
    }

    @Override
    public void loop() {
        applyPanelValues();

        // A changed gear ratio changes the conversion from servo angle to
        // turret angle. Reset PID history, but do NOT erase accumulated position.
        if (Math.abs(gearRatio - previousGearRatio) > 1e-9) {
            previousGearRatio = gearRatio;
            turret.resetController();
        }

        if (resetAngleToZero && !enabled) {
            turret.resetAngleTracking();
            resetAngleToZero = false;
        }

        if (enabled) {
            turret.trackAngle(Math.toRadians(targetAngleDegrees));
        } else {
            turret.stop();
        }

        panelsTelemetry.addData("Enabled", enabled);
        panelsTelemetry.addData("Target angle (deg)", targetAngleDegrees);
        panelsTelemetry.addData("Current angle (deg)", turret.getCurrentAngleDegrees());
        panelsTelemetry.addData("Angle error (deg)", turret.getAngleErrorDegrees());
        panelsTelemetry.addData("Servo power", turret.getServoPower());
        panelsTelemetry.addData("Encoder voltage", turret.getEncoderVoltage());

        panelsTelemetry.addData("Gear ratio (servo / turret)", gearRatio);
        panelsTelemetry.addData("Minimum angle (deg)", minAngleDegrees);
        panelsTelemetry.addData("Maximum angle (deg)", maxAngleDegrees);
        panelsTelemetry.addData("Maximum power", maxPower);
        panelsTelemetry.addData("Deadband (deg)", deadbandDegrees);

        panelsTelemetry.addData("P", p);
        panelsTelemetry.addData("I", i);
        panelsTelemetry.addData("D", d);
        panelsTelemetry.addData("F", f);
        panelsTelemetry.addData("Reset zero requested", resetAngleToZero);

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        enabled = false;
        turret.stop();
    }

    private void applyPanelValues() {
        // Keep the live values valid even if an accidental value is entered.
        Turret.turret_P = p;
        Turret.turret_I = i;
        Turret.turret_D = d;
        Turret.turret_F = f;

        Turret.TURRET_MAX_POWER = Math.max(0.0, Math.min(Math.abs(maxPower), 1.0));
        Turret.TURRET_DEADBAND_RADIANS = Math.toRadians(Math.abs(deadbandDegrees));
        Turret.TURRET_GEAR_RATIO = Math.max(Math.abs(gearRatio), 0.001);

        double min = Math.min(minAngleDegrees, maxAngleDegrees);
        double max = Math.max(minAngleDegrees, maxAngleDegrees);
        Turret.TURRET_MIN_ANGLE_RADIANS = Math.toRadians(min);
        Turret.TURRET_MAX_ANGLE_RADIANS = Math.toRadians(max);
    }
}
