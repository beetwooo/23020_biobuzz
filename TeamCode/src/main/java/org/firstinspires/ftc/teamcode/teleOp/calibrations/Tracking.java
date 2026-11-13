package org.firstinspires.ftc.teamcode.teleOp.calibrations;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Shooter;
import org.firstinspires.ftc.teamcode.features.shooter.Targeting;

@Configurable
@TeleOp(name = "Turret PID", group = "Calibrations")
public class Tracking extends OpMode {

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final Targeting targeting = new Targeting();
    private final Shooter shooter = new Shooter();

    public static boolean enabled = false;
    public static double targetAngleDegrees = 0; //default

    public static double P = Shooter.TURRET_P;
    public static double I = Shooter.TURRET_I;
    public static double D = Shooter.TURRET_D;
    public static double F = Shooter.TURRET_F;


    public static double maxPower = Shooter.TURRET_MAX_POWER;
    public static double deadbandDegrees =
            Math.toDegrees(Shooter.TURRET_DEADBAND_RADIANS);
    public static double gearRatio = Shooter.TURRET_GEAR_RATIO;

    private double previousGearRatio = gearRatio;




    @Override
    public void init(){

        targeting.init(hardwareMap);


    }

    @Override
    public void loop(){

        if(gearRatio != previousGearRatio){
            previousGearRatio = gearRatio;
        }

        if(enabled){

        }else{

        }

    }

    @Override
    public void stop(){
        enabled = false;
    }

}
