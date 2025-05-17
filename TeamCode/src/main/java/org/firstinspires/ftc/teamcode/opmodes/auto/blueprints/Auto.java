package org.firstinspires.ftc.teamcode.opmodes.auto.blueprints;

import static org.firstinspires.ftc.teamcode.architecture.Context.autoDriveTimer;
import static org.firstinspires.ftc.teamcode.architecture.Context.depositDriveTime;
import static org.firstinspires.ftc.teamcode.architecture.Context.intakeDriveTime;
import static org.firstinspires.ftc.teamcode.architecture.Context.type;
import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.tele.DRIVE_TYPE;

public abstract class Auto extends EnhancedOpMode {

    /**
     * Canvas is a builder. You can add lines, circles (robots), points, etc by saying c.OPERATION()
     * getOperations() is their version of build(), and we send that over as a TelemetryPacket to FtcDashboard
     * Actions.runBlocking() does this automatically, but we don't use that, so we're doing it ourselves
     * Why not Actions.runBlocking()?
     * Now that we don't have coroutines, everything must run through main loop (drivetrain updates, robot reads and writes)
     * this means that we need a way to continuously update each element without blocking the rest
     * so instead of doing Actions.runBlocking() once (which blocks everything else), we do action.run() every loop along with any other updates
     */
    protected Canvas c;

    /**
     * represents what's in specimenPath.createPath()... literally the logic for our entire auto in one Action
     */
    protected Action action;

    @Override
    protected void initialize() {
        c = new Canvas();
        Context.PIVOT_EXTENSION_RESET.hasReset = false;
        autoInit();
    }

    protected abstract void autoInit();

    @Override
    public void initializeLoop() {
        robot.read();
        robot.write();
        if (shouldUpdatePoseEstimate()) {
            robot.updatePoseEstimate();
        }
        initLoopHook();
    }

    protected void initLoopHook() { }

    protected boolean shouldUpdatePoseEstimate() {
        return false;
    }

    @Override
    public void onStart() {
        // set pose onStart() so you can move robot during init
        // add waitTime onStart() for AUTO_SELECTOR
        preOnStart();
        action = new SequentialAction(new SleepAction(Context.AUTO_SELECTOR.waitTime / 1000.0), action);
    }

    protected void preOnStart() { }

    @Override
    protected boolean automaticRobotWrite() {
        // this is the equivalent of putting robot.write() in primaryLoop. I just made it required so that we don't forget to write()
        return true;
    }

    @Override
    protected void primaryLoop() {

        robot.tel.addData("robot pose", robot.pose.position.x + ", " + robot.pose.position.y);
        robot.tel.addData("error", "X: " + (robot.pose.position.x - robot.followerTarget.position.x) + ", Y: " + (robot.pose.position.y - robot.followerTarget.position.y));

        TelemetryPacket packet = new TelemetryPacket();

        if (useCanvas()) {
            packet.fieldOverlay().getOperations().addAll(c.getOperations());
        }

        if (!action.run(packet)) {
            end();
        }

        if (!robot.isBusy) {
            robot.hold(packet);
        }

        updateTelemetry(packet);
    }

    protected boolean useCanvas() {
        return c != null;
    }

    protected void updateTelemetry(TelemetryPacket packet) {
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}