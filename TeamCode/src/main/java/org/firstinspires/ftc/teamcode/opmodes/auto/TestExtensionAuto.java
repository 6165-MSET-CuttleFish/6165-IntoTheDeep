package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.opmodes.auto.blueprints.Auto;
import org.firstinspires.ftc.teamcode.pathing.path_architecture.ActionBuilder;

@Autonomous(name = "C - Test Extension Auto")
public class TestExtensionAuto extends Auto {
    ActionBuilder actionBuilder;

    @Override
    protected void autoInit() {
        actionBuilder = new ActionBuilder();
        for (int i = 0; i < 100; i++) {
            actionBuilder
                    .addAction(robot.actions.highBasketExtensionActive(false))
                    .addDelayMs(2000)
                    .addAction(robot.actions.resetBasketAutoToIntake())
                    .addDelayMs(2000);
        }
        actionBuilder.addDelayMs(10000);
        action = actionBuilder.build();
    }
}