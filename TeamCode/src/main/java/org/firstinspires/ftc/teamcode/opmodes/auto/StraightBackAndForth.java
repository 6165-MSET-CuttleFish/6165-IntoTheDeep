package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.blueprints.Auto;
import org.firstinspires.ftc.teamcode.pathing.path_architecture.ActionBuilder;

@Autonomous(name = "C - StraightBackAndForth")
public class StraightBackAndForth extends Auto {
    ActionBuilder actionBuilder;

    @Override
    protected void autoInit() {
        actionBuilder = new ActionBuilder();
        for (int i = 0; i < 100; i++) {
            Action a = robot.actionBuilder(new Pose2d(0,0,0))
                    .lineToX(64)
                    .waitSeconds(2)
                    .lineToX(0)
                    .build();

            actionBuilder = actionBuilder.addAction(a);
        }

        action = actionBuilder.build();
    }
}