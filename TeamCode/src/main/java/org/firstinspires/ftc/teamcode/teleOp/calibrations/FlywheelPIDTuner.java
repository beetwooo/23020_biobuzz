package org.firstinspires.ftc.teamcode.teleOp.calibrations;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.features.shooter.Flywheel;

/**
 * Flywheel PID Tuner - controlled entirely from Panels (no gamepad).
 *
 * Panels > Configurables:
 *   FlywheelPIDTuner: flywheelEnabled, targetRpm
 *   ShooterConstants: P, I, D, F   (applied instantly while running)
 *
 * Panels > Graph: plot "Target RPM" and "Current RPM".
 *
 * Tuning:
 *   1. Set targetRpm, tick flywheelEnabled.
 *   2. Adjust F until Current RPM sits just under Target RPM.
 *   3. Raise P until it reaches the target quickly without bouncing.
 *   4. Only add I if it stays a little below target.
 *   5. Copy the final numbers into ShooterConstants.java (Panels changes are not saved).
 *
 * Finding NEAR_RPM / FAR_RPM: put the robot NEAR_DISTANCE inches from a goal,
 * change targetRpm until shots score, write that into NEAR_RPM. Repeat for FAR.
 */
@Configurable
@TeleOp(name = "Flywheel PID Tuner", group = "Calibrations")
public class FlywheelPIDTuner extends OpMode {

    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private final Flywheel flywheel = new Flywheel();

    public static boolean flywheelEnabled = false;
    public static double targetRpm = 3000;

    @Override
    public void init() {
        // Same bulk reads as TeleOp, so velocity readings behave the same.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        flywheel.init(hardwareMap);
        flywheelEnabled = false;   // always start with the flywheel off
    }

    @Override
    public void loop() {
        if (flywheelEnabled) {
            flywheel.setTargetRpm(targetRpm);
        } else {
            flywheel.stop();
        }
        flywheel.update();

        // Every number below can be plotted in the Panels Graph tab.
        panels.addData("Flywheel Enabled", flywheelEnabled);
        panels.addData("Target RPM", flywheel.getTargetRpm());
        panels.addData("Current RPM", flywheel.getCurrentRpm());
        panels.addData("Error RPM", flywheel.getErrorRpm());
        panels.addData("Motor Power x1000", flywheel.getPower() * 1000);
        panels.addData("Ready", flywheel.isReady());
        panels.addData("P", ShooterConstants.P);
        panels.addData("I", ShooterConstants.I);
        panels.addData("D", ShooterConstants.D);
        panels.addData("F", ShooterConstants.F);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        flywheelEnabled = false;
        flywheel.stop();
    }
}
