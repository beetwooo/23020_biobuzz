package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("LF");
                c.backLeftName.set("LB");
                c.frontRightName.set("RF");
                c.backRightName.set("RB");

                c.frontLeftDirection.set(DcMotor.Direction.REVERSE);
                c.backLeftDirection.set(DcMotor.Direction.REVERSE);
                c.frontRightDirection.set(DcMotor.Direction.FORWARD);
                c.backRightDirection.set(DcMotor.Direction.FORWARD);
            }
    );

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
}