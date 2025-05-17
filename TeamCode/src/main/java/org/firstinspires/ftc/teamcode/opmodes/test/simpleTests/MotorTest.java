package org.firstinspires.ftc.teamcode.opmodes.test.simpleTests;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;

/**
 * see CRServoTest for comments
 */
@Config
@TeleOp(name = "B - Motor Test", group = "Simple Testing")
public class MotorTest extends EnhancedOpMode {
    public static String name = "intake";
    public static double power = -1;
    public static boolean RunWithEncoder = false;
    DcMotorEx motor;
    String lastName = name;

    @Override
    protected void initialize() {
        motor = hardwareMap.get(DcMotorEx.class, name);

    }

    @Override
    protected void primaryLoop() {
        if (RunWithEncoder) motor.setMode(RUN_USING_ENCODER);

        motor.setPower(power);
        if (!lastName.equals(name)) {
            motor = hardwareMap.get(DcMotorEx.class, name);
            lastName = name;
        }
        robot.tel.addData("Current", motor.getCurrent(CurrentUnit.AMPS));
        robot.tel.addData("Velocity", motor.getVelocity());
    }

    @Override
    protected boolean automaticRobotWrite() {
        return false;
    }
}
