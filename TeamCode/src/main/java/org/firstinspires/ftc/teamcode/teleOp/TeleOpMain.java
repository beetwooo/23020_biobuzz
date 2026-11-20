
package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.Field;
import org.firstinspires.ftc.teamcode.features.Mecanum;
import org.firstinspires.ftc.teamcode.features.intake.Intake;
import org.firstinspires.ftc.teamcode.features.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.features.shooter.ShotSolution;
import org.firstinspires.ftc.teamcode.features.shooter.ChassisAim;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public abstract class TeleOpMain extends OpMode {

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

    private ShotSolution shot;

    @Override
    public void init() {

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
        double aimTurn = chassisAim.calculate(
                robotPose,
                follower.velocity().omega,
                goalX,
                goalY,
                0.0
        );

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

        // START: reset to the starting pose. Only press when the robot is really there.
        mecanum.resetPose(gamepad1.startWasPressed(), startingPose);

        // Hold LEFT BUMPER to spin up the flywheel.
        if (gamepad1.left_bumper && robotPose != null) {
            shot = flywheel.aimAt(
                    robotPose.x(),
                    robotPose.y(),
                    goalX,
                    goalY);
        } else {
            flywheel.stop();
            shot = null;
        }
        flywheel.update();

        // RIGHT TRIGGER = intake.
        // LEFT TRIGGER = feed/shoot, only when the flywheel is ready AND the robot faces the goal.
        // Hold Y with LEFT TRIGGER to skip the aim check (e.g. if odometry is off).
        boolean aimOk = chassisAim.isAimed() || gamepad1.y;

        if (gamepad1.right_trigger_pressed) {
            intake.setState(Intake.State.INTAKING);
        } else if (gamepad1.left_trigger_pressed && flywheel.isReady() && aimOk) {
            intake.setState(Intake.State.OUTTAKING);
        } else {
            intake.setState(Intake.State.IDLE);
        }
        intake.update();

        if (robotPose != null) {
            telemetry.addData("Robot X", robotPose.x());
            telemetry.addData("Robot Y", robotPose.y());
            telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        }
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Active Goal", activeGoal);
        telemetry.addData("Target Goal", aimingAtTopGoal ? "TOP" : "BOTTOM");
        telemetry.addData("Goal X", goalX);
        telemetry.addData("Goal Y", goalY);
        telemetry.addData("Driver Heading", driverHeadingDegrees);
        telemetry.addData("Auto Aiming", autoAiming);
        telemetry.addData("Aimed", chassisAim.isAimed());
        telemetry.addData("Aim Error (deg)", chassisAim.getHeadingErrorDegrees());
        telemetry.addData("Aim Turn Power", chassisAim.getTurnPower());
        telemetry.addData("Flywheel State", flywheel.getState());
        telemetry.addData("Target Velocity", flywheel.getTargetVelocity());
        telemetry.addData("Current Velocity", flywheel.getCurrentVelocity());
        telemetry.addData("Velocity Error", flywheel.getVelocityError());
        telemetry.addData("Flywheel Ready", flywheel.isReady());
        if (shot != null) {
            telemetry.addData("Shot Distance", shot.getDistance());
            telemetry.addData("Shot Valid", shot.isValid());
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setState(Intake.State.IDLE);
        intake.update();

        flywheel.stop();
        follower.stop();
    }
}
