package org.firstinspires.ftc.teamcode.opmodes.test.other;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;

@Config
@TeleOp(name = "D - Ada")
public class Ada extends EnhancedOpMode {

    @Override
    public void initializeLoop() {
        robot.read();
        robot.write();
    }

    @Override
    public void onStart() {
        robot.runAction(robot.actions.Ada());
    }

    @Override
    protected boolean automaticRobotWrite() {
        return true;
    }
}