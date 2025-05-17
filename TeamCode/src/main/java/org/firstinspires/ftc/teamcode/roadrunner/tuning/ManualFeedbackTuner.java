package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.architecture.Robot;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.Drive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.unused.TankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.unused.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.unused.TwoDeadWheelLocalizer;

@Autonomous
public class ManualFeedbackTuner extends EnhancedOpMode {
    public static double DISTANCE = 64;

    @Override
    public void primaryLoop() {
        Actions.runBlocking(
                robot.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(DISTANCE)
                        .lineToX(0)
                        .build());
    }

    @Override
    protected boolean automaticRobotWrite() {
        return true;
    }
}
