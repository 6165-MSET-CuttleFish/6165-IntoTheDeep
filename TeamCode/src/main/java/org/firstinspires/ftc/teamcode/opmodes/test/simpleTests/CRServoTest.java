package org.firstinspires.ftc.teamcode.opmodes.test.simpleTests;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;

@TeleOp(name = "B - CR Servo Test", group = "Simple Testing")
@Config
public class CRServoTest extends EnhancedOpMode {
    public static String name = "servo";
    public static double power = 0;
    CRServo cr_servo;
    String lastName = name;

    @Override
    protected void initialize() {
        cr_servo = hardwareMap.get(CRServo.class, name);
    }

    @Override
    protected void primaryLoop() {
        cr_servo.setPower(power);

        /** if you ever change the name of the cr_servo, it'll update in real time */
        if (!lastName.equals(name)) {

            cr_servo = hardwareMap.get(CRServo.class, name);
            lastName = name;
        }

        /**
         * Note that this will print ALL CURRENT FROM EACH HUB, not each individual servo
         * To find current draw of servo, you'll just have to graph the correct hub's current and watch the spike
         */

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            robot.tel.addData(module.getDeviceName(), module.getGpioBusCurrent(CurrentUnit.AMPS));
        }
    }

    /**
     * robot writes would override our cr_servo writes otherwise, that's why this function exists
     */
    @Override
    protected boolean automaticRobotWrite() {
        return false;
    }
}
