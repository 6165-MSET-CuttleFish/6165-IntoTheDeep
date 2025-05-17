package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.modules.Hang;
import org.firstinspires.ftc.teamcode.architecture.modules.IntakeSpecimen;
import org.firstinspires.ftc.teamcode.architecture.modules.PivotExtension;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;

@Config
@TeleOp(name = "B - Module Test")
public class ModuleTest extends EnhancedOpMode {

    public static double pivot = PivotExtension.PivotState.HIGH_BASKET_ACTIVE.getValue();
    public static double extension = PivotExtension.ExtensionState.RESET.getValue();
    public static double wrist = IntakeSpecimen.WristState.INIT.getValue();
    public static double claw = IntakeSpecimen.ClawState.CLOSED.getValue();
    public static double intake = IntakeSpecimen.PowerState.OFF.getValue();
    public static double leftSweeper = IntakeSpecimen.LeftSweeperState.IN.getValue();
    public static double rightSweeper = IntakeSpecimen.RightSweeperState.IN.getValue();
    public static double leftPto = Hang.LeftPTOState.RAISED.getValue();
    public static double rightPto = Hang.RightPTOState.RAISED.getValue();
    public static double leftPitch = Hang.LeftPitchState.INIT.getValue();
    public static double rightPitch = Hang.RightPitchState.INIT.getValue();
    public static double tape = Hang.TapeState.GROUND.getValue();

    public static double bothPowerDT = 0;
    public static boolean bothDT = false;
    public static double powerRightDT = 0;
    public static double powerLeftDT = 0;

    @Override
    public void initialize() {
        Context.read = false;

    }

    public void initializeLoop() {
        robot.read();
    }


    @Override
    public void primaryLoop() {
        robot.velocity = robot.updatePoseEstimate();

        robot.pivotExtension.pivotTarget = pivot;
        robot.pivotExtension.extensionTarget = extension;

        robot.intakeSpecimen.wristPosition = wrist;
        robot.intakeSpecimen.clawPosition = claw;
        robot.intakeSpecimen.intakePower = intake;
        robot.intakeSpecimen.leftSweeperPosition = leftSweeper;
        robot.intakeSpecimen.rightSweeperPosition = rightSweeper;

        robot.hang.leftPtoPosition = leftPto;
        robot.hang.rightPtoPosition = rightPto;
        robot.hang.leftPitchPosition = leftPitch;
        robot.hang.rightPitchPosition = rightPitch;
        robot.hang.tapeTarget = tape;

        if (bothDT) {
            robot.setDrivetrainPowers(bothPowerDT, bothPowerDT, bothPowerDT, bothPowerDT);
        } else {
            robot.setDrivetrainPowers(powerLeftDT, powerLeftDT, powerRightDT, powerRightDT);
        }

        robot.pivotExtension.telemetryUpdate();
        robot.intakeSpecimen.telemetryUpdate();

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