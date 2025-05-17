package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.Robot;
import org.firstinspires.ftc.teamcode.architecture.RobotActions;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;

@TeleOp(name="C - Localization Test")
public class LocalizationTest extends EnhancedOpMode {

        @Override
        public void primaryLoop() {
            robot.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            robot.updatePoseEstimate();

            robot.tel.addData("x", robot.pose.position.x);
            robot.tel.addData("y", robot.pose.position.y);
            robot.tel.addData("heading (deg)", Math.toDegrees(robot.pose.heading.toDouble()));

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), robot.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            Robot.WRITE_TOGGLES.drivetrainWrite = true;
            Robot.WRITE_TOGGLES.robotWrite = true;
        }

    @Override
    protected boolean automaticRobotWrite() {
        return false;
    }
}
