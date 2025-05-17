package org.firstinspires.ftc.teamcode.architecture.opmodes;

import static org.firstinspires.ftc.teamcode.architecture.Robot.CONTROLLER;
import static org.firstinspires.ftc.teamcode.architecture.Robot.WRITE_TOGGLES;
import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.architecture.modules.IntakeSpecimen;
import org.firstinspires.ftc.teamcode.architecture.modules.PivotExtension;
import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.Robot;
import org.firstinspires.ftc.teamcode.architecture.tele.ACTION_MODE;

/**
 * We should have no need for LinearOpMode anymore...
    * There's no harm in initializing robot, no one ever said you have to use it
    * There's no way for null pointer exceptions to occur if everything is always initialized automatically... right?
 */
public abstract class EnhancedOpMode extends LinearOpMode {

    /**
     * boolean end used to stop opMode when end() is called
     */
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    boolean end = false;
    double oldTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotLog.e("sus enhanced 0: " + timer2.milliseconds());
        initializeStatics();

        initialize();

        /** Note: there is no robot.read() and robot.write() in initialize... it must be called in the opmode */
        while (opModeInInit()) {
            /** with FTCLib gamepads, you must read the value of all the buttons every loop to update them */
            for (KeyReader reader: robot.keyReaders) {
                reader.readValue();
            }

            if (end) {
                return;
            }
            // 2.5-3 ms per bulk read
            // 2.5-3 ms per encoder/digital/...
            for (LynxModule m: hardwareMap.getAll(LynxModule.class)) {
                m.clearBulkCache();
            }

            robot.tel.addData("robot pose", robot.pose.position.x + ", " + robot.pose.position.y);
            robot.tel.addData("robot heading", robot.pose.heading.toDouble());
            /** automatic telemetry updating */
            robot.tel.update();

            /** for any stuff to run in the initializeLoop in the opMode */
            initializeLoop();
        }

        /** renamed linearOpMode() to onStart() bc that makes more sense */
        onStart();

        while (opModeIsActive()) {
            timer2.reset();
            if (end) {
                return;
            }

            for (KeyReader reader: robot.keyReaders) {
                reader.readValue();
            }
          //  RobotLog.e("sus enhanced after KEY READ: " + timer2.milliseconds());

            for (LynxModule m: hardwareMap.getAll(LynxModule.class)) {
                m.clearBulkCache();
            }
          //  RobotLog.e("sus enhanced after CACHE: " + timer2.milliseconds());

            /** automatic robot updates and writes */

            robot.read();
          //  RobotLog.e("sus enhanced after READ: " + timer2.milliseconds());

            primaryLoop();
           // RobotLog.e("sus enhanced after PRIMARY LOOP: " + timer2.milliseconds());

            if (automaticRobotWrite()) {
                robot.write();
            }

           // RobotLog.e("sus enhanced after WRITE: " + timer2.milliseconds());

            /** automatic telemetry updating */
            robot.tel.addData("robot pose", robot.pose.position.x + ", " + robot.pose.position.y);
            robot.tel.addData("robot heading", robot.pose.heading.toDouble());
            robot.tel.addData("loop time", (timer.milliseconds()) + " ms");
            robot.tel.addData("Total Curernt:", robot.getCurrent());
          //  RobotLog.e("fl power" + robot.fl.getPower());
            robot.tel.update();

            timer.reset();

            while (timer.milliseconds() < Context.manualLoopTimeDelay && opModeIsActive()) { }

            /** for any stuff to run in the loop of the opMode */
        }

        onEnd();
    }

    /**
     * pose save ALWAYS happens first
        * if you never set pos, it'll take the last robot pos
            * this is ideal for pose storage in tele
        * if you do, it'll override this every time because robot is ALWAYS initialized first
     * If we ever have color detection for intake, this is where we would initialize the color of our bot
     */
    protected void initializeStatics() throws InterruptedException {

        Context.isLimelight = false;
        Context.isColorSensor = false;
        Context.read = true;
        Context.actionMode = ACTION_MODE.INTAKE;
        Context.readyForTapeMeasure = false;
        Context.hangSequenceCount = 0;

        Pose2d startPose = robot == null ? new Pose2d(0,0,0) : robot.pose;
        robot = new Robot(this, startPose);

        for (LynxModule m: hardwareMap.getAll(LynxModule.class)) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /**
     * none are abstract anymore
        * figured that we don't really NEED any of them to be abstract but yall can change if u want
     */
    protected void initialize(){}
    protected void primaryLoop(){}
    protected void onStart(){}
    protected void onEnd(){}
    protected void initializeLoop(){}

    /**
     * in some opModes you don't want robot writes to override manual writes, so can set automaticRobotWrite() to false
     */
    protected abstract boolean automaticRobotWrite();
    public void end() {
        end = true;
    }
}
