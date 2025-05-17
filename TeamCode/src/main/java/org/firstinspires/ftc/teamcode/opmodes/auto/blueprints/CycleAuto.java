package org.firstinspires.ftc.teamcode.opmodes.auto.blueprints;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.modules.IntakeSpecimen;
import org.firstinspires.ftc.teamcode.pathing.BasketPath;
import org.firstinspires.ftc.teamcode.pathing.CyclePath;

public class CycleAuto extends Auto {

    private double firstValue;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    protected void autoInit() {
        Context.isLimelight = true;
//        Context.isColorSensor = true;
        CyclePath basketPath = new CyclePath(Context.POSES.basketStart);
        action = basketPath.createPath();
        firstValue = robot.intakeSpecimen.getState(IntakeSpecimen.WristState.class).getValue();
        if (timer.milliseconds() > 3000) {
            robot.intakeSpecimen.getState(IntakeSpecimen.WristState.class).setValue(0.5);
        }
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