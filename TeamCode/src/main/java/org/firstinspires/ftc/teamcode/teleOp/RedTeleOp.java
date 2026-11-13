package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.Field;

@TeleOp(name = "TeleOp Red", group = "23020 Biobuzz TeleOp")
public class RedTeleOp extends TeleOpMain {

    @Override
    protected Field.Goal getGoal() {
        return Field.Goal.RED;
    }
}
