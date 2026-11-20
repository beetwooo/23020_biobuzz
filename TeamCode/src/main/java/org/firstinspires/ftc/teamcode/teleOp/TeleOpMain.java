
package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.Field;
import org.firstinspires.ftc.teamcode.features.Mecanum;
import org.firstinspires.ftc.teamcode.features.intake.Mouth;
import org.firstinspires.ftc.teamcode.features.shooter.ShooterForTrajectory;
import org.firstinspires.ftc.teamcode.features.shooter.Targeting;
import org.firstinspires.ftc.teamcode.features.shooter.Trajectory;
import org.firstinspires.ftc.teamcode.features.shooter.test2;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public abstract class TeleOpMain extends OpMode {

    private Follower follower;

    private final Mouth mouth = new Mouth();
    private final Targeting turret = new Targeting();

    private final test2 test2 = new test2();

    private Mecanum mecanum;

    private Field.Goal activeGoal;

    private double goalX;
    private double goalY;
    private double driverHeadingDegrees;
    private Pose startingPose;
    private Pose robotPose;

    private final PoseFactory p = PoseFactory.degrees();

    protected abstract Field.Goal getGoal();

    private final ShooterForTrajectory shooterForTrajectory = new ShooterForTrajectory();

    private Trajectory trajectory;

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
            startingPose = p.of(
                    Field.RED_START_X,
                    Field.RED_START_Y,
                    Field.RED_START_HEADING_DEGREES
            );
        } else {
            startingPose = p.of(
                    Field.BLUE_START_X,
                    Field.BLUE_START_Y,
                    Field.BLUE_START_HEADING_DEGREES
            );
        }

        follower.setPose(startingPose);
        robotPose = follower.pose();

        boolean topGoal =
                startingPose.x() > 72
                        || startingPose.y() > 72;

        if (activeGoal == Field.Goal.RED) {
            goalX = topGoal ? Field.RED_GOAL_X_TOP : Field.RED_GOAL_X_BOTTOM;
            goalY = topGoal ? Field.RED_GOAL_Y_TOP : Field.RED_GOAL_Y_BOTTOM;
        } else {
            goalX = topGoal ? Field.BLUE_GOAL_X_TOP : Field.BLUE_GOAL_X_BOTTOM;
            goalY = topGoal ? Field.BLUE_GOAL_Y_TOP : Field.BLUE_GOAL_Y_BOTTOM;
        }

        mouth.init(hardwareMap);
        turret.init(hardwareMap);
        mecanum = new Mecanum(follower);
        shooterForTrajectory.init(hardwareMap);
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

        // Gamepad -> Pedro convention (forward +, left +, counter-clockwise +).
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // Auto aim (hold RIGHT BUMPER). test2 already returns a Pedro-convention
        // turn (+ = counter-clockwise), so it is NOT negated like the stick.
        if (gamepad1.right_bumper) {
            double aimTurn = test2.calculate(robotPose, goalX, goalY, 0.0);

            // Driver can still override with the right stick.
            if (Math.abs(gamepad1.right_stick_x) <= 0.12) {
                turn = aimTurn;
            }
        }

        mecanum.fieldCentric(
                forward,
                strafe,
                turn,
                Math.toRadians(driverHeadingDegrees)
        );

        // START: reset to the starting pose. Only press when the robot is really there.
        mecanum.resetPose(gamepad1.startWasPressed(), startingPose);

        // Hold LEFT BUMPER to spin up the flywheel.
        if (gamepad1.left_bumper && robotPose != null) {
            trajectory = shooterForTrajectory.aimAt(
                    robotPose.x(),
                    robotPose.y(),
                    goalX,
                    goalY);
        } else {
            shooterForTrajectory.stop();
            trajectory = null;
        }
        shooterForTrajectory.update();

        // RIGHT TRIGGER = intake, LEFT TRIGGER = feed/shoot (only when the flywheel is ready).
        if (gamepad1.right_trigger_pressed) {
            mouth.setState(Mouth.State.INTAKING);
        } else if (gamepad1.left_trigger_pressed && shooterForTrajectory.isReady()) {
            mouth.setState(Mouth.State.OUTTAKING);
        } else {
            mouth.setState(Mouth.State.IDLE);
        }
        mouth.update();

        if (robotPose != null) {
            telemetry.addData("Robot X", robotPose.x());
            telemetry.addData("Robot Y", robotPose.y());
            telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        }
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Active Goal", activeGoal);
        telemetry.addData("Goal X", goalX);
        telemetry.addData("Goal Y", goalY);
        telemetry.addData("Driver Heading", driverHeadingDegrees);
        telemetry.addData("Aim Error (deg)", test2.getHeadingErrorDegrees());
        telemetry.addData("Shooter State", shooterForTrajectory.getState());
        telemetry.addData("Target Velocity", shooterForTrajectory.getTargetVelocity());
        telemetry.addData("Current Velocity", shooterForTrajectory.getCurrentVelocity());
        telemetry.addData("Velocity Error", shooterForTrajectory.getVelocityError());
        telemetry.addData("Shooter Ready", shooterForTrajectory.isReady());
        if (trajectory != null) {
            telemetry.addData("Shot Distance", trajectory.getDistance());
            telemetry.addData("Trajectory Valid", trajectory.isValid());
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        mouth.setState(Mouth.State.IDLE);
        mouth.update();

        shooterForTrajectory.stop();
        turret.stop();
        follower.stop();
    }
}
