
package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.Field;
import org.firstinspires.ftc.teamcode.features.Mecanum;
import org.firstinspires.ftc.teamcode.features.intake.Lip;
import org.firstinspires.ftc.teamcode.features.intake.Mouth;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TELEOP_TEST", group = "TeleOp")
public class TeleOpMain extends OpMode {

    public Follower follower;

    private Mouth mouth = new Mouth();
    private Lip lip = new Lip();
    private Mecanum mecanum;

    private double goalX;
    private double goalY;
    private Pose startingPose;
    double driverHeadingDegrees;
    private boolean usingAutoPose;
    private boolean previousPoseResetCombo;
    private boolean previousTurretResetCombo;
    private Pose robotPose;
    private Field.Goal activeGoal;
    protected abstract Field.Goal getGoal();

    @Override
    public void init() {


        activeGoal = getGoal();

        if(robotPose.x() > 72 || robotPose.y() > 72){
            goalX = activeGoal == Field.Goal.RED
                    ? Field.RED_GOAL_X_TOP : Field.BLUE_GOAL_X_TOP;
            goalY = activeGoal == Field.Goal.RED
                    ? Field.RED_GOAL_Y_TOP : Field.BLUE_GOAL_Y_TOP;
        }else{
            goalX = activeGoal == Field.Goal.RED
                    ? Field.RED_GOAL_X_BOTTOM : Field.BLUE_GOAL_X_BOTTOM;
            goalY = activeGoal == Field.Goal.RED
                    ? Field.RED_GOAL_Y_BOTTOM : Field.BLUE_GOAL_Y_BOTTOM;
        }

        

        double startX = activeGoal == Field.Goal.RED
                ? FieldConst.RED_START_X : FieldConst.BLUE_START_X;
        double startY = activeGoal == Field.Goal.RED
                ? FieldConst.RED_START_Y : FieldConst.BLUE_START_Y;
        double startHeading = activeGoal == Field.Goal.RED
                ? FieldConst.RED_START_HEADING_DEGREES
                : FieldConst.BLUE_START_HEADING_DEGREES;
        driverHeadingDegrees = activeGoal == Field.Goal.RED
                ? FieldConst.RED_DRIVER_HEADING_DEGREES
                : FieldConst.BLUE_DRIVER_HEADING_DEGREES;
        startingPose = new Pose(
                startX,
                startY,
                Math.toRadians(startHeading));
        Pose autoEndPose = PoseStorage.getAutoPose(activeGoal);
        if (autoEndPose != null) {
            startingPose = autoEndPose;
            usingAutoPose = true;
        }



        follower = Constants.create(hardwareMap);
        mouth.init(hardwareMap);
        lip.init(hardwareMap);
        mecanum = new Mecanum(follower);



    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        double ly = gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        boolean startWasPressed = gamepad1.startWasPressed();
        boolean bWasPressed = gamepad1.bWasPressed();

        mecanum.fieldCentric(ly, lx, rx);

        mecanum.resetPose(startWasPressed, bWasPressed);

        robotPose = follower.pose();

        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());

        telemetry.addData(
                "Robot Heading",
                Math.toDegrees(robotPose.heading())
        );

        telemetry.update();
    }
}
