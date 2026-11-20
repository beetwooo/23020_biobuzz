
package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.Field;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.features.Mecanum;
import org.firstinspires.ftc.teamcode.features.intake.Intake;
import org.firstinspires.ftc.teamcode.features.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.features.shooter.ChassisAim;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public abstract class TeleOpMain extends OpMode {

    // Panels: telemetry goes to Panels AND the Driver Station.
    // The Panels Graph tab plots any number sent with panels.addData.
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Follower follower;

    private final Intake intake = new Intake();

    private final ChassisAim chassisAim = new ChassisAim();

    private Mecanum mecanum;

    private Field.Goal activeGoal;

    private double goalX;
    private double goalY;
    private boolean aimingAtTopGoal;
    private double driverHeadingDegrees;
    private Pose startingPose;
    private Pose robotPose;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    protected abstract Field.Goal getGoal();

    private final Flywheel flywheel = new Flywheel();
    private boolean flywheelOn = false;

    private boolean firing = false;
    private final ElapsedTime fireTimer = new ElapsedTime();

    @Override
    public void init() {

        // Bulk reads: read all motor encoders/velocities on each hub in one go.
        // AUTO refreshes the cache by itself, so nothing else is needed in loop().
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        activeGoal = getGoal();

        driverHeadingDegrees =
                activeGoal == Field.Goal.RED
                        ? Field.RED_DRIVER_HEADING_DEGREES
                        : Field.BLUE_DRIVER_HEADING_DEGREES;

        // Pedro 3: build the follower first.
        follower = Constants.create(hardwareMap);

        if (activeGoal == Field.Goal.RED) {
            startingPose = poseFactory.of(
                    Field.RED_START_X,
                    Field.RED_START_Y,
                    Field.RED_START_HEADING_DEGREES
            );
        } else {
            startingPose = poseFactory.of(
                    Field.BLUE_START_X,
                    Field.BLUE_START_Y,
                    Field.BLUE_START_HEADING_DEGREES
            );
        }

        selectGoal(startingPose);

        // No autonomous: TeleOp always starts from the Field.java starting pose,
        // so place the robot exactly there before pressing INIT.
        follower.setPose(startingPose);
        robotPose = follower.pose();

        intake.init(hardwareMap);
        mecanum = new Mecanum(follower);
        flywheel.init(hardwareMap);
    }

    /**
     * Biobuzz: each alliance has 2 goals.
     * Robot in the upper half (y > 72) aims at the TOP goal,
     * otherwise (y <= 72) it aims at the BOTTOM goal.
     */
    private void selectGoal(Pose pose) {
        if (pose == null) {
            return;
        }

        aimingAtTopGoal = pose.y() > 72;

        if (activeGoal == Field.Goal.RED) {
            goalX = aimingAtTopGoal ? Field.RED_GOAL_X_TOP : Field.RED_GOAL_X_BOTTOM;
            goalY = aimingAtTopGoal ? Field.RED_GOAL_Y_TOP : Field.RED_GOAL_Y_BOTTOM;
        } else {
            goalX = aimingAtTopGoal ? Field.BLUE_GOAL_X_TOP : Field.BLUE_GOAL_X_BOTTOM;
            goalY = aimingAtTopGoal ? Field.BLUE_GOAL_Y_TOP : Field.BLUE_GOAL_Y_BOTTOM;
        }
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        // Pedro 3: update the follower EVERY loop (reads the Pinpoint and sends motor powers).
        // Without this the robot never drives and the pose never changes.
        follower.update();
        robotPose = follower.pose();

        // Re-pick the goal every loop from where the robot is now.
        selectGoal(robotPose);

        // Gamepad -> Pedro convention (forward +, left +, counter-clockwise +).
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // Aim is calculated every loop so we always know whether the robot is aimed.
        // ChassisAim returns a Pedro-convention turn (+ = counter-clockwise),
        // so it is NOT negated like the stick.
        double aimTurn = chassisAim.calculate(follower, goalX, goalY);

        // Auto aim (hold RIGHT BUMPER). Driver can still override with the right stick.
        boolean autoAiming = gamepad1.right_bumper
                && Math.abs(gamepad1.right_stick_x) <= 0.12;

        if (autoAiming) {
            // No HOLD while aiming, so small aim corrections are not ignored.
            mecanum.fieldCentricNoHold(
                    forward,
                    strafe,
                    aimTurn,
                    Math.toRadians(driverHeadingDegrees)
            );
        } else {
            mecanum.fieldCentric(
                    forward,
                    strafe,
                    turn,
                    Math.toRadians(driverHeadingDegrees)
            );
        }

        // START + X (together): reset to the starting pose. Fires once per press.
        // Only press when the robot is really at the starting pose.
        boolean resetPressed = gamepad1.xWasPressed() && gamepad1.start;
        mecanum.resetPose(resetPressed, startingPose);

        // Flywheel toggle: A = on (stays on between shots), B = off.
        // While on, the speed follows the distance to the goal.
        if (gamepad1.aWasPressed()) flywheelOn = true;
        if (gamepad1.bWasPressed()) flywheelOn = false;

        double goalDistance = Math.hypot(goalX - robotPose.x(), goalY - robotPose.y());

        if (flywheelOn) {
            flywheel.spinUpForDistance(goalDistance);
        } else {
            flywheel.stop();
        }
        flywheel.update();

        // RIGHT TRIGGER = intake.
        // LEFT TRIGGER = feed/shoot. A shot only STARTS when the flywheel is ready AND the
        // robot faces the goal (hold Y to skip the aim check, e.g. if odometry is off).
        // Once started, it keeps feeding for FIRE_TIME seconds even though the RPM dips
        // as the ball goes through (one motor dips a lot). Keep holding to fire the next
        // ball as soon as the flywheel is ready again. Releasing the trigger stops feeding.
        boolean aimOk = chassisAim.isAimed() || gamepad1.y;

        if (firing && fireTimer.seconds() >= ShooterConstants.FIRE_TIME) {
            firing = false;   // firing window finished
        }
        if (!firing && gamepad1.left_trigger_pressed && flywheel.isReady() && aimOk) {
            firing = true;    // start a new firing window
            fireTimer.reset();
        }

        if (gamepad1.right_trigger_pressed) {
            firing = false;
            intake.setState(Intake.State.INTAKING);
        } else if (firing && gamepad1.left_trigger_pressed) {
            intake.setState(Intake.State.OUTTAKING);
        } else {
            firing = false;
            intake.setState(Intake.State.IDLE);
        }
        intake.update();

        if (robotPose != null) {
            panels.addData("Robot X", robotPose.x());
            panels.addData("Robot Y", robotPose.y());
            panels.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        }
        panels.addData("Follower Mode", follower.mode());
        panels.addData("Active Goal", activeGoal);
        panels.addData("Target Goal", aimingAtTopGoal ? "TOP" : "BOTTOM");
        panels.addData("Goal X", goalX);
        panels.addData("Goal Y", goalY);
        panels.addData("Driver Heading", driverHeadingDegrees);
        panels.addData("Auto Aiming", autoAiming);
        panels.addData("Aimed", chassisAim.isAimed());
        panels.addData("Aim Error (deg)", chassisAim.getHeadingErrorDegrees());
        panels.addData("Aim Turn Power", chassisAim.getTurnPower());
        panels.addData("Goal Distance (in)", goalDistance);
        panels.addData("Flywheel On", flywheelOn);
        panels.addData("Firing", firing);
        panels.addData("Flywheel Target RPM", flywheel.getTargetRpm());
        panels.addData("Flywheel Current RPM", flywheel.getCurrentRpm());
        panels.addData("Flywheel Ready", flywheel.isReady());
        panels.addData("Shot Speeds Calibrated", Flywheel.isCalibrated());
        // Graph tab: every number sent with panels.addData can be graphed
        // (Flywheel Target/Current RPM, Aim Error...). This one shows the firing windows.
        panels.addData("Firing x1000", firing ? 1000 : 0);

        panels.update(telemetry);
    }

    @Override
    public void stop() {
        intake.setState(Intake.State.IDLE);
        intake.update();

        flywheel.stop();
        follower.stop();
    }
}
