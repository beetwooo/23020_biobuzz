
package org.firstinspires.ftc.teamcode.teleOp.calibrations;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
@TeleOp(name = "Shooting PID", group = "Calibrations")
public class Shooting extends OpMode {

    private final TelemetryManager panelsTelemetry =
            PanelsTelemetry.INSTANCE.getTelemetry();

    private DcMotorEx flywheel;

    // =====================================
    // PANELS VARIABLES
    // =====================================

    public static boolean flywheelEnabled = false;

    public static double p =
            org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_P;

    public static double i =
            org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_I;

    public static double d =
            org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_D;

    public static double f =
            org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_F;

    // Encoder ticks per second
    public static double tar_vel = 0;

    private double lastP;
    private double lastI;
    private double lastD;
    private double lastF;

    // =====================================
    // READY CHECK
    // =====================================

    private final ElapsedTime readyTimer = new ElapsedTime();

    private boolean inTolerance = false;
    private boolean ready = false;

    // =====================================
    // GAMEPAD EDGE DETECTION
    // =====================================

    private boolean previousA;
    private boolean previousUp;
    private boolean previousDown;
    private boolean previousY;

    // =====================================
    // INIT
    // =====================================

    @Override
    public void init() {

        flywheel = hardwareMap.get(
                DcMotorEx.class,
                org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_NAME
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

        flywheel.setPower(0);

        flywheelEnabled = false;

        applyPIDF();
    }

    // =====================================
    // APPLY PIDF
    // =====================================

    private void applyPIDF() {

        p = Math.max(0, p);
        i = Math.max(0, i);
        d = Math.max(0, d);
        f = Math.max(0, f);

        PIDFCoefficients coefficients =
                new PIDFCoefficients(p, i, d, f);

        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                coefficients
        );

        lastP = p;
        lastI = i;
        lastD = d;
        lastF = f;

        // Copy calibrated values into constants.
        org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_P = p;
        org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_I = i;
        org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_D = d;
        org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_F = f;
    }

    // =====================================
    // LOOP
    // =====================================

    @Override
    public void loop() {

        // Update PIDF when changed in Panels.
        if (p != lastP
                || i != lastI
                || d != lastD
                || f != lastF) {

            applyPIDF();
        }

        // =================================
        // GAMEPAD CONTROLS
        // =================================

        if (gamepad1.a && !previousA) {
            flywheelEnabled = !flywheelEnabled;
        }

        if (gamepad1.dpad_up && !previousUp) {
            tar_vel += 50;
        }

        if (gamepad1.dpad_down && !previousDown) {
            tar_vel -= 50;
        }

        if (gamepad1.y && !previousY) {
            tar_vel = 0;
        }

        previousA = gamepad1.a;
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
        previousY = gamepad1.y;

        // =================================
        // VELOCITY LIMIT
        // =================================

        double maxVelocity =
                org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_MAX_RPM
                        / 60.0
                        * org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_TPR;

        tar_vel = Range.clip(tar_vel, 0, maxVelocity);

        double commandedVelocity =
                flywheelEnabled ? tar_vel : 0;

        // =================================
        // MOTOR CONTROL
        // =================================

        if (commandedVelocity > 0) {

            flywheel.setVelocity(commandedVelocity);

        } else {

            flywheel.setPower(0);
        }

        // =================================
        // VELOCITY FEEDBACK
        // =================================

        double currentVelocity = flywheel.getVelocity();

        double error = commandedVelocity - currentVelocity;

        ready = false;

        if (flywheelEnabled
                && commandedVelocity > 0
                && Math.abs(error)
                <= org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_READY_TOLERANCE) {

            if (!inTolerance) {

                readyTimer.reset();

                inTolerance = true;
            }

            ready = readyTimer.seconds()
                    >= org.firstinspires.ftc.teamcode.constants.Shooter.FLYWHEEL_READY_TIME;

        } else {

            inTolerance = false;

            readyTimer.reset();
        }

        // =================================
        // PANELS TELEMETRY
        // =================================

        panelsTelemetry.addData("Flywheel Enabled", flywheelEnabled);

        panelsTelemetry.addData("Target Velocity", commandedVelocity);

        panelsTelemetry.addData("Current Velocity", currentVelocity);

        panelsTelemetry.addData("Velocity Error", error);

        panelsTelemetry.addData("Motor Power", flywheel.getPower());

        panelsTelemetry.addData(
                "Encoder Ticks",
                flywheel.getCurrentPosition()
        );

        panelsTelemetry.addData("Ready", ready);

        panelsTelemetry.addData("P", p);
        panelsTelemetry.addData("I", i);
        panelsTelemetry.addData("D", d);
        panelsTelemetry.addData("F", f);

        panelsTelemetry.update(telemetry);
    }

    // =====================================
    // STOP
    // =====================================

    @Override
    public void stop() {

        flywheelEnabled = false;

        ready = false;

        if (flywheel != null) {
            flywheel.setPower(0);
        }
    }
}
