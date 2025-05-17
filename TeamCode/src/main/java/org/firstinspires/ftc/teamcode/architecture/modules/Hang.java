package org.firstinspires.ftc.teamcode.architecture.modules;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;
import org.firstinspires.ftc.teamcode.architecture.modules.moduleUtil.Module;
import org.firstinspires.ftc.teamcode.architecture.modules.moduleUtil.ModuleState;

@Config
public class Hang extends Module {

    public static boolean telemetryToggle = false;
    public static boolean readCurrent = false;

    public boolean autoPTO = false;
    public boolean isHangActive = false;

    public EnhancedServo leftPitch, rightPitch, leftPTO, rightPTO;
    public double tapeTarget, leftPtoPosition, rightPtoPosition, leftPitchPosition, rightPitchPosition;
    public double leftCurrent, rightCurrent;

    public enum TapeState implements ModuleState {
        GROUND(0);

        private double value;

        TapeState(double value) {
            this.value = value;
        }

        @Override
        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = value;
        }
    }

    public enum LeftPitchState implements ModuleState {
        INIT(0.6);

        private double value;

        LeftPitchState(double value) {
            this.value = value;
        }

        @Override
        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = MathUtils.clamp(value, 0.4, 0.76);
        }
    }

    public enum RightPitchState implements ModuleState {
        INIT(0.6);

        private double value;

        RightPitchState(double value) {
            this.value = value;
        }

        @Override
        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = MathUtils.clamp(value, 0.3, 0.66);
        }
    }

    public enum LeftPTOState implements ModuleState {
        DOWN(1),
        RAISED(0);

        private double value;

        LeftPTOState(double value) {
            this.value = value;
        }

        @Override
        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = value;
        }
    }

    public enum RightPTOState implements ModuleState {
        DOWN(0),
        RAISED(1);

        private double value;

        RightPTOState(double value) {
            this.value = value;
        }

        @Override
        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = value;
        }
    }

    public Hang(HardwareMap hardwareMap) {
        super(true, telemetryToggle);

        // Initialize servos
        leftPitch = new EnhancedServo(hardwareMap, "leftPitch");
        rightPitch = new EnhancedServo(hardwareMap, "rightPitch");
        leftPTO = new EnhancedServo(hardwareMap, "leftPTO");
        rightPTO = new EnhancedServo(hardwareMap, "rightPTO");
        rightPitch.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    protected void write() {
        if (autoPTO) {
            if (robot.fl.getPower() > 0 && robot.bl.getPower() > 0) {
                leftPTO.setPosition(LeftPTOState.DOWN.value);
            } else {
                leftPTO.setPosition(LeftPTOState.RAISED.value);
            }

            if (robot.fr.getPower() > 0 && robot.br.getPower() > 0) {
                rightPTO.setPosition(RightPTOState.DOWN.value);
            } else {
                rightPTO.setPosition(RightPTOState.RAISED.value);
            }
        } else {
            leftPTO.setPosition(leftPtoPosition);
            rightPTO.setPosition(rightPtoPosition);
        }

        leftPitch.setPosition(leftPitchPosition);
        rightPitch.setPosition(rightPitchPosition);
    }

    @Override
    protected void read() {
        leftPtoPosition = getState(LeftPTOState.class).getValue();
        rightPtoPosition = getState(RightPTOState.class).getValue();
        tapeTarget = getState(TapeState.class).getValue();
        leftPitchPosition = getState(LeftPitchState.class).getValue();
        rightPitchPosition = getState(RightPitchState.class).getValue();

        if (readCurrent) {
            leftCurrent = robot.bl.getCurrent(CurrentUnit.AMPS) + robot.fl.getCurrent(CurrentUnit.AMPS);
            rightCurrent = robot.br.getCurrent(CurrentUnit.AMPS) + robot.fr.getCurrent(CurrentUnit.AMPS);
        }
    }

    @Override
    protected void initInternalStates() {
        setInternalStates(TapeState.GROUND, LeftPitchState.INIT, RightPitchState.INIT,
                LeftPTOState.RAISED, RightPTOState.RAISED);
    }

    @Override
    public void telemetryUpdate() {
        if (telemetryToggle) {
            super.telemetryUpdate();
            robot.tel.addData("Hang Tape state", getState(TapeState.class));
            robot.tel.addData("Hang Left Pitch state", getState(LeftPitchState.class));
            robot.tel.addData("Hang Right Pitch state", getState(RightPitchState.class));
            robot.tel.addData("Hang Left PTO state", getState(LeftPTOState.class));
            robot.tel.addData("Hang Right PTO state", getState(RightPTOState.class));
            robot.tel.addData("tapeTargetPosition", tapeTarget);
            robot.tel.addData("leftTapePosition", robot.bl.getCurrentPosition());
            robot.tel.addData("rightTapePosition", robot.br.getCurrentPosition());
            robot.tel.addData("leftPower", (robot.bl.getPower() + robot.fl.getPower()) / 2);
            robot.tel.addData("rightPower", (robot.br.getPower() + robot.fr.getPower()) / 2);

            if (readCurrent) {
                robot.tel.addData("leftCurrent", leftCurrent);
                robot.tel.addData("rightCurrent", rightCurrent);
            }

            robot.tel.addData("leftPitchPosition", leftPitchPosition);
            robot.tel.addData("rightPitchPosition", rightPitchPosition);
            robot.tel.addData("leftPtoPosition", leftPtoPosition);
            robot.tel.addData("rightPtoPosition", rightPtoPosition);
            robot.tel.addData("Hang Left Encoder", robot.pivotExtension.pivot.getCurrentPosition());
            robot.tel.addData("Hang Right Encoder", robot.fl.getCurrentPosition());
        }
    }

    @Override
    protected void updateInternalStatus() {
        status = Status.IDLE;
    }
}
