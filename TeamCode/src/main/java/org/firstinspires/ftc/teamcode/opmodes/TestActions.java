package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;

@Config
@TeleOp(name = "B - Test Actions")
public class TestActions extends EnhancedOpMode {

    public static int index = 0;

    double lastIndex = -1;

    @Override
    protected boolean automaticRobotWrite() {
        return true;
    }

    @Override
    protected void primaryLoop() {

        if (index != lastIndex) {
            switch (index) {
                case 0:
                    robot.runAction(robot.actions.highBasketExtensionActive(true));
                case 1:
                    robot.runAction(robot.actions.intake(false));
                case 2:
                    robot.runAction(robot.actions.specimenPickup());
                case 3:
                    robot.runAction(robot.actions.specimenScore(true));
            }
            lastIndex = index;
        }

    }
}

