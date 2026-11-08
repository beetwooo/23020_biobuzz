package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("FL");
                c.backLeftName.set("BL");
                c.frontRightName.set("FR");
                c.backRightName.set("BR");

                c.frontLeftDirection.set(DcMotor.Direction.REVERSE);
                c.backLeftDirection.set(DcMotor.Direction.REVERSE);
                c.frontRightDirection.set(DcMotor.Direction.FORWARD);
                c.backRightDirection.set(DcMotor.Direction.FORWARD);
            });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
           c.name.set("pinpoint");
           c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
           c.xPodOffset.set(-2.8801000399852366);
           c.yPodOffset.set(-5.27652770515502);
           c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
           c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
           c.globalDistanceUnit.set(DistanceUnit.CM);
           c.offsetUnits.set(DistanceUnit.INCH);

           });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.17320823655421172);
                Controller secondaryTranslationalForward = Controller.proportional(0.06399581519746154);
                Controller primaryTranslationalLateral = Controller.proportional(0.29571073255705665);
                Controller secondaryTranslationalLateral = Controller.proportional(0.10925721414353383);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.005173757319677942));
                c.brake.set(Controller.proportionalFeedforward(0.004397693721726251));

                c.headingFeedback.set(Controller.proportional(6.666391566141709));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06952081822053731, 0.004010396957531123));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0942387590578199, 0.04861883696631769));
                c.quadraticBrakeCoefficients.set(Matrix.diag(7.376560522554141E-4, 8.3357518709766E-4));

                c.maxAchievableForwardVelocity.set(155.94886660206433);
                c.maxAchievableStrafeVelocity.set(117.07789765719151);
                c.naturalForwardDeceleration.set(59.88311876548025);
                c.naturalStrafeDeceleration.set(186.59545412386285);
            }
    );




}