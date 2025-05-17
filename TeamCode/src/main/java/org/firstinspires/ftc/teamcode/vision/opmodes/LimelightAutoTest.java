package org.firstinspires.ftc.teamcode.vision.opmodes;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.modules.Limelight;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.pathing.BasketPath;
import org.firstinspires.ftc.teamcode.vision.Pipelines.Color;

@Autonomous(name = "C - Limelight Auto Test")
public class LimelightAutoTest extends EnhancedOpMode {

    private final Canvas canvas = new Canvas();
    private Action limelightAction;

    private boolean isRunning = true;

    boolean firstWrite = false;

    @Override
    public void initialize() {
        Context.isColorSensor = true;
        Context.isLimelight = true;
        Context.color = Color.BLUE;
    }

    @Override
    public void initializeLoop() {
        robot.read();
        robot.write();
        double current = robot.intakeSpecimen.intake.getCurrent(CurrentUnit.AMPS);
        robot.tel.addData("current", current);
        robot.updatePoseEstimate();
    }

    @Override
    public void onStart() {
        robot.pose = BasketPath.cyclePose;
        limelightAction = new Limelight.searchForSampleRegionals(new Vector2d(BasketPath.cyclePose.position.x, BasketPath.cyclePose.position.y));
        limelightAction.preview(canvas);
    }

    @Override
    protected boolean automaticRobotWrite() {
        return true;
    }

    @Override
    protected void primaryLoop() {
        Context.isColorSensor = true;
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

        if (isRunning && !limelightAction.run(packet)) {
            isRunning = false;
            robot.runAction(robot.actions.highBasketExtensionActive(false));
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}