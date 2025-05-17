package org.firstinspires.ftc.teamcode.opmodes.test.simpleTests;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;

/**
 * see CRServoTest for comments
 */
@Config
@TeleOp(name = "B - Servo Test", group = "Simple Testing")
public class ServoTest extends EnhancedOpMode {
    public static String name = "claw";
    public static double position = 0.81;
    public static boolean pwm = true;
    Servo servo;
    String lastName = name;

    @Override
    protected void initialize() {
        servo = hardwareMap.get(Servo.class, name);
    }

    @Override
    protected void primaryLoop() {
        servo.setPosition(position);
        if (!lastName.equals(name)) {
            servo = hardwareMap.get(Servo.class, name);
            lastName = name;
        }
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.getGpioBusCurrent(CurrentUnit.AMPS);
        }

        robot.tel.addData("Total current", robot.getCurrent());
    }

    @Override
    protected boolean automaticRobotWrite() {
        return false;
    }
}
