package org.firstinspires.ftc.teamcode.vision.opmodes;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.modules.IntakeSpecimen;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.vision.Pipelines.Color;

@TeleOp(name = "C - Intake Vision Test")
public class IntakeVisionTest extends EnhancedOpMode {

    @Override
    protected void initialize() {
        Context.isColorSensor = true;
    }

    @Override
    protected void primaryLoop() {
        if (robot.intakeSpecimen.checkAgainstColors(Color.NEUTRAL, Context.color)) {
            robot.intakeSpecimen.setState(IntakeSpecimen.PowerState.INTAKE);
        } else if (robot.intakeSpecimen.checkAgainstColors(Color.NONE)) {
            robot.intakeSpecimen.setState(IntakeSpecimen.PowerState.OFF);
        } else {
            robot.intakeSpecimen.setState(IntakeSpecimen.PowerState.EXTAKE);
        }

        robot.tel.addData("color", robot.intakeSpecimen.activeColor.name());
    }

    @Override
    protected boolean automaticRobotWrite() {
        return false;
    }
}
