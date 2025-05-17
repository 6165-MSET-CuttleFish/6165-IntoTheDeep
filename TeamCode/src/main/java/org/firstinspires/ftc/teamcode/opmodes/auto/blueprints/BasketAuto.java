package org.firstinspires.ftc.teamcode.opmodes.auto.blueprints;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;
import static org.firstinspires.ftc.teamcode.pathing.BasketPath.cyclePose;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.Robot;
import org.firstinspires.ftc.teamcode.architecture.modules.IntakeSpecimen;
import org.firstinspires.ftc.teamcode.pathing.BasketPath;
import org.firstinspires.ftc.teamcode.roadrunner.Drive;

public class BasketAuto extends Auto {

    private double firstValue;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean firstInit = true;
    private double firstY;

    @Override
    protected void autoInit() {
        Drive.PARAMS.axialVelGain = 1;
        Drive.PARAMS.axialGain = 20;
        Context.isLimelight = true;
//        Context.isColorSensor = true;
        BasketPath basketPath = new BasketPath(Context.POSES.basketStart);
        action = basketPath.createPath();
        firstValue = robot.intakeSpecimen.getState(IntakeSpecimen.WristState.class).getValue();
        if (timer.milliseconds() > 3000) {
            robot.intakeSpecimen.getState(IntakeSpecimen.WristState.class).setValue(0.5);
        }
    }

    @Override
    protected void initLoopHook() {
        if (firstInit) {
            firstY = cyclePose.position.y;
            firstInit = false;
        }
        if (Robot.CONTROLLER.DPAD_LEFT1.wasJustPressed() || Robot.CONTROLLER.DPAD_LEFT2.wasJustPressed()) {
            cyclePose = new Pose2d(cyclePose.position.x, cyclePose.position.y - 0.5, cyclePose.heading.toDouble());
        }
        if (Robot.CONTROLLER.DPAD_RIGHT1.wasJustPressed() || Robot.CONTROLLER.DPAD_RIGHT2.wasJustPressed()) {
            cyclePose = new Pose2d(cyclePose.position.x, cyclePose.position.y + 0.5, cyclePose.heading.toDouble());
        }
        robot.tel.addData("Default Cycle Pose Y", firstY);
        robot.tel.addData("Cycle Pose Y", cyclePose.position.y);
    }

    @Override
    protected void preOnStart() {
        robot.intakeSpecimen.setState(IntakeSpecimen.WristState.INIT);
        robot.intakeSpecimen.getState(IntakeSpecimen.WristState.class).setValue(firstValue);
        robot.pose = Context.POSES.basketStart;
    }

    @Override
    protected boolean useCanvas() {
        return false;
    }
}