package org.firstinspires.ftc.teamcode.opmodes.test.simpleTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;

import java.util.HashMap;
import java.util.Map;

@Config
@TeleOp(name = "B - Multiple Motor Test", group = "Simple Testing")
public class MultipleMotorTest extends EnhancedOpMode {
    public static String[] motorNames = {"motor1", "motor2", "motor3", "motor4"};
    public static double[] motorPowers = {0, 0, 0, 0};

    private Map<String, DcMotorEx> motors = new HashMap<>();

    @Override
    protected void initialize() {
        for (String name : motorNames) {
            try {
                motors.put(name, hardwareMap.get(DcMotorEx.class, name));
            } catch (Exception ignored) {

            }

        }
    }

    @Override
    protected void primaryLoop() {
        for (int i = 0; i < motorNames.length; i++) {
            try {
                String name = motorNames[i];
                double power = motorPowers[i];
                DcMotorEx motor = motors.get(name);

                if (motor != null) {
                    motor.setPower(power);
                } else {
                    telemetry.addData("Error", "Motor " + name + " not found");
                }
            } catch (Exception ignored) {

            }

        }

        updateTelemetry();
    }

    private void updateTelemetry() {
        for (int i = 0; i < motorNames.length; i++) {
            String name = motorNames[i];
            double power = motorPowers[i];
            DcMotorEx motor = motors.get(name);

            if (motor != null) {
                telemetry.addData(name + " Power", power);
                telemetry.addData(name + " Position", motor.getCurrentPosition());
                telemetry.addData(name + " Position", motor.getCurrent(CurrentUnit.AMPS));
            }
        }
        telemetry.update();
    }

    @Override
    protected boolean automaticRobotWrite() {
        return false;
    }
}
