package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.modules.PivotExtension;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;

@Config
@TeleOp(name = "B - Willisa Test")
public class WillisaTest extends EnhancedOpMode {

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void initializeLoop() {
        robot.read();
        robot.write();
    }

    @Override
    public void onStart() {
        robot.pivotExtension.setState(PivotExtension.PivotState.HIGH_BASKET_ACTIVE);
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 3000) {
            robot.read();
            robot.write();
        }
    }


    @Override
    public void primaryLoop() {
        robot.pivotExtension.setState(PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE);

        robot.tel.addData("current", robot.getCurrent());
    }

    @Override
    public void onEnd() {
        Context.read = true;
    }

    @Override
    protected boolean automaticRobotWrite() {
        return true;
    }


}