package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.architecture.Context.*;
import static org.firstinspires.ftc.teamcode.architecture.Robot.CONTROLLER;
import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;
import static org.firstinspires.ftc.teamcode.architecture.tele.DRIVE_TYPE.AUTO_BASKET;
import static org.firstinspires.ftc.teamcode.architecture.tele.DRIVE_TYPE.AUTO_SUBMERSIBLE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.RobotActions;
import org.firstinspires.ftc.teamcode.architecture.modules.Hang;
import org.firstinspires.ftc.teamcode.architecture.modules.IntakeSpecimen;
import org.firstinspires.ftc.teamcode.architecture.modules.PivotExtension;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.tele.ACTION_MODE;
import org.firstinspires.ftc.teamcode.architecture.tele.DRIVE_TYPE;
import org.firstinspires.ftc.teamcode.vision.Pipelines.Color;

@Config
@TeleOp(name = "A - Tele")
public class Tele extends EnhancedOpMode {
    // Constants
    private static final double MAX_CURRENT = 5.5;
    private static final double STICK_DEADZONE = 0.3;
    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final int SAFE_EXTAKE_DELAY_MS = 1000;

    // Hang state
    private int hangSequenceCount = 0;
    private boolean firstHangLoop = true;
    private boolean isIdleHangPower = false;

    // Intake/Extake state
    private boolean clawIntake = false;
    private boolean slowExtake = true;
    private boolean safeExtake = false;
    private boolean shortExtend = false;
    private boolean retracted = false;

    private boolean firstExtension = false;

    // Basket state
    private boolean isHighBasket = true;
    private boolean slowDeposit = false;
    private double slowDepositExtend = 3.5;
    private double depositExtend = 0;

    // Robot control
    private RobotActions actions;

    // Timers
    private ElapsedTime resetTimer;
    private ElapsedTime safeExtakeTimer;

    // Rumble effect for controller feedback
    private final Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, 400)
            .addStep(0, 0, 200)
            .addStep(1, 1, 200)
            .build();

    @Override
    public void initialize() {
        autoDriveTimer = new ElapsedTime();
        type = DRIVE_TYPE.MANUAL;
        actions = robot.actions;
        robot.pivotExtension.setState(PivotExtension.PivotState.INIT);
        robot.pivotExtension.resetTimer.reset();
        robot.hang.autoPTO = false;
    }

    @Override
    public void initializeLoop() { }

    @Override
    public void onStart() {
        resetTimer = new ElapsedTime();
        safeExtakeTimer = new ElapsedTime();
        robot.intakeSpecimen.setState(IntakeSpecimen.WristState.SAMPLE_INTAKE);
    }

    @Override
    protected boolean automaticRobotWrite() { return true; }

    @Override
    public void primaryLoop() {
        handleDrivetrainMovement();
        handleActionMode();
        robot.tel.addData("actionMode", actionMode.name());
    }

    //region Action Mode Handlers
    private void handleActionMode() {
        switch (actionMode) {
            case HANG:
                handleHangActions();
                break;
            case INTAKE:
                handleIntakeActions();
                break;
            case BASKET:
                handleBasketActions();
                break;
            case SPECIMEN_PICKUP:
                handleSpecimenPickupActions();
                break;
            case SPECIMEN_DEPOSIT:
                handleSpecimenDepositActions();
                break;
        }
    }

    private void handleHangActions() {
        if (CONTROLLER.B2.wasJustPressed()) {
            if (robot.hang.getState(Hang.LeftPTOState.class).equals(Hang.LeftPTOState.RAISED)) {
                robot.hang.setState(Hang.LeftPTOState.DOWN);
                robot.hang.setState(Hang.RightPTOState.DOWN);
            } else {
                robot.hang.setState(Hang.LeftPTOState.RAISED);
                robot.hang.setState(Hang.RightPTOState.RAISED);
            }
        }

        if (CONTROLLER.A1.wasJustPressed() && hangSequenceCount <= 1) {
            hangSequenceCount++;
            if (hangSequenceCount == 1) {
                robot.runAction(actions.hangLow());
                hangSequenceCount++;
            }
        }

        if (CONTROLLER.RB2.wasJustPressed()) {
            if (hangSequenceCount == 2) {
                hangSequenceCount++;
                robot.runAction(actions.unhookFirstBarHang());
                robot.hang.leftPitch.pwm(false);
                robot.hang.rightPitch.pwm(false);
            } else if (hangSequenceCount == 3) {
                hangSequenceCount++;
                robot.pivotExtension.setState(PivotExtension.PivotState.INIT);
                robot.hang.leftPitch.pwm(false);
                robot.hang.rightPitch.pwm(false);
            }
        }

        if (CONTROLLER.DPAD_UP2.wasJustPressed()) {
            if (isIdleHangPower) {
                isIdleHangPower = false;
                robot.setDrivetrainPowers(0, 0, 0, 0);
            } else {
                isIdleHangPower = true;
                double power = 0.25;
                robot.setDrivetrainPowers(power, power, power, power);
            }
        }

        handleHangMovement();
        handleHangPitchAdjustment();

        if (hangSequenceCount == 0) {
            driveMove();
        }

        robot.tel.addData("hangSequenceCount", hangSequenceCount);

        if (CONTROLLER.B1.wasJustPressed() && !firstHangLoop) {
            hangSequenceCount = 0;
            robot.hang.isHangActive = false;
            robot.runAction(actions.resetFromHang());
            actionMode = ACTION_MODE.INTAKE;
        }

        if (firstHangLoop) firstHangLoop = false;
    }

    private void handleHangMovement() {
        if (hangSequenceCount >= 1) {
            double rightPower = -gamepad2.left_stick_y;
            double leftPower = -gamepad2.right_stick_y;

            /* if (robot.hang.leftCurrent > MAX_CURRENT || robot.hang.rightCurrent > MAX_CURRENT) {
                double multiplier = Math.max(MAX_CURRENT / robot.hang.leftCurrent, MAX_CURRENT / robot.hang.rightCurrent);
                rightPower = rightPower * multiplier;
                leftPower = leftPower * multiplier;
            } */
            if (!isIdleHangPower) {
                if (CONTROLLER.A2.isDown()) {
                    robot.setDrivetrainPowers(-rightPower, -rightPower, -rightPower, -rightPower);
                } else {
                    robot.setDrivetrainPowers(-leftPower, -leftPower, -rightPower, -rightPower);
                }
            }
        }
    }

    private void handleHangPitchAdjustment() {
        if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            double adjustment = pitchSpeed * gamepad2.left_trigger;
            Hang.LeftPitchState.INIT.setValue(Hang.LeftPitchState.INIT.getValue() + adjustment);
            Hang.RightPitchState.INIT.setValue(Hang.RightPitchState.INIT.getValue() + adjustment);
        }

        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            double adjustment = pitchSpeed * gamepad2.right_trigger;
            Hang.LeftPitchState.INIT.setValue(Hang.LeftPitchState.INIT.getValue() - adjustment);
            Hang.RightPitchState.INIT.setValue(Hang.RightPitchState.INIT.getValue() - adjustment);
        }
    }

    private void handleIntakeActions() {
        // Handle control toggles
        if (CONTROLLER.A2.wasJustPressed()) {
            shortExtend = !shortExtend;
            gamepad2.runRumbleEffect(rumbleEffect);
        }

        if (robot.intakeSpecimen.countColor > 8) {
            robot.runAction(actions.light());
        }

        if (CONTROLLER.DPAD_UP2.wasJustPressed()) {
            clawIntake = !clawIntake;
            gamepad2.runRumbleEffect(rumbleEffect);
        }

        if (CONTROLLER.LSB2.wasJustPressed()) {
            slowExtake = !slowExtake;
        }


        // Handle intake and extake operations\
        if ((robot.pivotExtension.pivotPosition < 25) || robot.pivotExtension.getState(PivotExtension.PivotState.class).equals(PivotExtension.PivotState.INIT)) {
            handleIntakeOperations();
        }

        // Handle speed controls
        handleSpeedControls();

        if (CONTROLLER.RB1.wasJustPressed()) {
            robot.runAction(actions.specimenScore(false));
            actionMode = ACTION_MODE.SPECIMEN_PICKUP;
        }

        // Handle transitions to other modes
        handleBasketTransition();

    }

    private void handleIntakeOperations() {
        // Handle intake control
        if (CONTROLLER.RB2.isDown()) {
            if (clawIntake) {
                robot.intakeSpecimen.setState(IntakeSpecimen.ClawState.CLOSED);
            } else {
                robot.runAction(actions.justIntake());
            }
        } else if (safeExtake && safeExtakeTimer.milliseconds() > SAFE_EXTAKE_DELAY_MS &&
                robot.intakeSpecimen.getState(IntakeSpecimen.PowerState.class).equals(IntakeSpecimen.PowerState.OFF)) {
            robot.intakeSpecimen.setState(IntakeSpecimen.PowerState.HOLDING);
        }

        // Handle rumble on overcurrent
        if (robot.intakeSpecimen.intake.isOverCurrent()) {
            gamepad2.runRumbleEffect(rumbleEffect);
        }

        // Handle dropdown operations
        handleDropdownOperations();

        // Handle extake operations
        if (CONTROLLER.LB2.isDown() ||
                robot.intakeSpecimen.checkAgainstColors(Context.color == Color.BLUE ? Color.RED : Color.BLUE)) {
            robot.intakeSpecimen.setState(
                    slowExtake ? IntakeSpecimen.PowerState.SLOW_EXTAKE : IntakeSpecimen.PowerState.YEETMORE
            );
            ninja = 1;
            ninjaStrafe = 1;
        }

        // Handle sweeper toggling
        if (CONTROLLER.X2.wasJustPressed()) {
            boolean sweepersOut = robot.intakeSpecimen.getState(IntakeSpecimen.LeftSweeperState.class)
                    .equals(IntakeSpecimen.LeftSweeperState.OUT);
            robot.intakeSpecimen.setState(sweepersOut ?
                    IntakeSpecimen.LeftSweeperState.IN : IntakeSpecimen.LeftSweeperState.OUT);
            robot.intakeSpecimen.setState(sweepersOut ?
                    IntakeSpecimen.RightSweeperState.IN : IntakeSpecimen.RightSweeperState.OUT);
        }
    }

    private void handleDropdownOperations() {
        if (CONTROLLER.LT2.isDown()) {
            robot.runAction(actions.justTopDropdown());
            robot.runAction(actions.sweep());
        } else if (CONTROLLER.LT2.wasJustReleased()) {
            robot.runAction(actions.resetToIntakeHover());
        }

        if (CONTROLLER.B2.wasJustPressed() && !retracted) {
            PivotExtension.ExtensionState.MANUAL.setValue(0);
            robot.pivotExtension.setState(PivotExtension.ExtensionState.MANUAL);
            retracted = true;
        } else if (CONTROLLER.B2.wasJustPressed()) {
            PivotExtension.ExtensionState.MANUAL.setValue(PivotExtension.ExtensionState.INTAKE_EXTENSION.getValue());
            robot.pivotExtension.setState(PivotExtension.ExtensionState.MANUAL);
            retracted = false;
        }

        if (CONTROLLER.RT2.isDown()) {
            if (clawIntake) {
                robot.runAction(actions.clawDropdown());
                gamepad2.runRumbleEffect(rumbleEffect);
            } else if (shortExtend) {
                robot.runAction(actions.shortDropdown());
                gamepad2.runRumbleEffect(rumbleEffect);
            } else {
                robot.runAction(actions.justDropdown());
                robot.runAction(actions.sweep());
            }
        } else if (CONTROLLER.RT2.wasJustReleased()) {
            gamepad2.runRumbleEffect(rumbleEffect);
            robot.runAction(actions.resetToIntakeHover());
            ninja = 1;
            ninjaStrafe = 1;
        }
    }

    private void handleBasketTransition() {
        if (CONTROLLER.LB1.wasJustPressed()) {
            if (isHighBasket) {
                if (clawIntake) {
                    robot.runAction(actions.highBasketExtensionClaw(false));
                } else {
                    robot.runAction(actions.highBasketExtensionActive(true));
                }
            } else {
                if (clawIntake) {
                    robot.runAction(actions.lowBasketExtensionClaw());
                } else {
                    robot.runAction(actions.lowBasketExtensionIntake());
                }
            }
            actionMode = ACTION_MODE.BASKET;
        }


    }

    private void handleSpeedControls() {
        if (CONTROLLER.RT1.isDown()) {
            // Speed controls handled in driveMove()
        } else if (CONTROLLER.RT1.wasJustReleased()) {
            ninja = 1;
            ninjaStrafe = 1;
        }
    }

    private void handleBasketActions() {
        // Toggle slow deposit
        if (CONTROLLER.Y2.wasJustPressed() && !slowDeposit) {
            slowDeposit = true;
            gamepad2.runRumbleEffect(rumbleEffect);
        }

        if (CONTROLLER.RT2.wasJustPressed() && robot.pivotExtension.pivotPosition > 90) {
            depositExtend += 1;
            gamepad2.runRumbleEffect(rumbleEffect);
            PivotExtension.ExtensionState.MANUAL.setValue(
                    PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE.getValue() + depositExtend);
            robot.pivotExtension.setState(PivotExtension.ExtensionState.MANUAL);
        } else if (CONTROLLER.LT2.wasJustPressed() && robot.pivotExtension.pivotPosition > 90) {
            depositExtend -= 1;
            gamepad2.runRumbleEffect(rumbleEffect);
            PivotExtension.ExtensionState.MANUAL.setValue(
                    PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE.getValue() + depositExtend);
            robot.pivotExtension.setState(PivotExtension.ExtensionState.MANUAL);
        }

        IntakeSpecimen.PowerState powerState = slowDeposit ?
                IntakeSpecimen.PowerState.SLOW_SCORE : IntakeSpecimen.PowerState.SCORE;
        if (CONTROLLER.LT1.wasJustPressed() || CONTROLLER.RB1.wasJustPressed()) {
            if (robot.pivotExtension.pivotPosition > 90) robot.intakeSpecimen.setState(powerState);
            if (slowDeposit && isHighBasket) {
                PivotExtension.ExtensionState.MANUAL.setValue(
                        PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE.getValue() + depositExtend + slowDepositExtend);
                robot.pivotExtension.setState(PivotExtension.ExtensionState.MANUAL);
            }
        } else if (CONTROLLER.LT1.wasJustReleased()) {
            safeExtake = true;
            safeExtakeTimer.reset();
            robot.runAction(actions.intake(true));
            actionMode = ACTION_MODE.INTAKE;
            ninja = 1;
            ninjaStrafe = 1;
        } else if (CONTROLLER.RB1.wasJustReleased()) {
            // Transition to specimen pickup mode
            robot.runAction(actions.basketScoreToSpecimenPickup());
            actionMode = ACTION_MODE.SPECIMEN_PICKUP;
            ninja = 1;
            ninjaStrafe = 1;
        }
    }

    private void handleSpecimenPickupActions() {
        // Handle transitions to other modes

        if (CONTROLLER.RT2.wasJustPressed() || CONTROLLER.RT1.wasJustPressed()) {
            robot.runAction(robot.actions.specimenToIntake());
            actionMode = ACTION_MODE.INTAKE;
        }
        if (CONTROLLER.RB1.wasJustPressed()) {
            gamepad2.runRumbleEffect(rumbleEffect);
            robot.runAction(actions.specimenPickupTele());
            actionMode = ACTION_MODE.SPECIMEN_DEPOSIT;
        }

        if (CONTROLLER.A2.wasJustPressed()) {
            if (robot.intakeSpecimen.getState(IntakeSpecimen.ClawState.class).equals(IntakeSpecimen.ClawState.CLOSED)) {
                robot.intakeSpecimen.setState(IntakeSpecimen.ClawState.OPEN);
            } else {
                robot.intakeSpecimen.setState(IntakeSpecimen.ClawState.CLOSED);
            }
        }
    }

    private void handleSpecimenDepositActions() {
        // Extension offset adjustments
        if (CONTROLLER.A1.wasJustPressed()) {
            PivotExtension.extensionOffset -= 0.5;
        }

        if (CONTROLLER.Y1.wasJustPressed()) {
            PivotExtension.extensionOffset += 0.5;
        }

        if (CONTROLLER.RB1.wasJustPressed()) {
            robot.runAction(actions.specimenScore(false));
            actionMode = ACTION_MODE.SPECIMEN_PICKUP;
        }
    }

    //endregion

    //region Drivetrain Handling
    private void handleDrivetrainMovement() {
        if (CONTROLLER.DPAD_RIGHT2.wasJustPressed()) {
            robot.intakeSpecimen.manualWristOffset -= robot.intakeSpecimen.wristPosition - 0.02 <= 0 ? 0 : 0.02;
        }

        if (CONTROLLER.DPAD_LEFT2.wasJustPressed()) {
            robot.intakeSpecimen.manualWristOffset += robot.intakeSpecimen.wristPosition + 0.02 >= 1 ? 0 : 0.02;
        }

        // Handle flipper toggling
        if (CONTROLLER.DPAD_DOWN2.wasJustPressed() || CONTROLLER.DPAD_UP1.wasJustPressed()) {
            robot.intakeSpecimen.setState(robot.intakeSpecimen.getState(IntakeSpecimen.FlipperState.class)
                    .equals(IntakeSpecimen.FlipperState.UP) ?
                    IntakeSpecimen.FlipperState.DOWN : IntakeSpecimen.FlipperState.UP);
        }

        if (!actionMode.equals(ACTION_MODE.HANG)) {
            // Handle basket height toggle
            if (CONTROLLER.RSB2.wasJustPressed()) {
                isHighBasket = !isHighBasket;
                if (actionMode.equals(ACTION_MODE.BASKET)) {
                    robot.runAction(isHighBasket ? actions.highBasketExtensionActive(false) : actions.lowBasketExtensionIntake());
                }
                gamepad2.runRumbleEffect(rumbleEffect);
            }

            // Handle different drive types
            switch (type) {
                case AUTO_BASKET:
                    handleAutoDrive(-1, type);
                    break;
                case AUTO_SPECIMEN:
                case AUTO_SUBMERSIBLE:
                    handleAutoDrive(1, type);
                    break;
                default:
                    driveMove();
                    break;
            }

            if (robot.pivotExtension.getState(PivotExtension.PivotState.class).equals(PivotExtension.PivotState.INTAKE)) {
                isColorSensor = false; //usecolor
            } else {
                isColorSensor = false;
            }

            // Handle hang mode activation
            if (CONTROLLER.B1.wasJustPressed()) {
                robot.hang.isHangActive = true;
                firstHangLoop = true;
                robot.runAction(actions.hangSetup());
                actionMode = ACTION_MODE.HANG;
            }
        }
    }

    private void handleAutoDrive(int power, DRIVE_TYPE driveType) {
        robot.setDrivetrainPowers(power, power, power, power);

        double driveTimeLimit = driveType.equals(AUTO_BASKET) ?
                intakeDriveTime : (driveType.equals(AUTO_SUBMERSIBLE) ? depositDriveTime : specimenDriveTime);

        if (autoDriveTimer.milliseconds() > driveTimeLimit) {
            type = DRIVE_TYPE.MANUAL;
        }
    }

    public void driveMove() {
        double x = calculateMovement(gamepad1.left_stick_y, ninja);
        double y = calculateMovement(gamepad1.left_stick_x, ninjaStrafe);
        double rx = calculateRotation(gamepad1.right_stick_x, ninja);

        double[] highSpeedAdjustment = adjustForHighSpeed(x, y, rx);
        x = highSpeedAdjustment[0];
        y = highSpeedAdjustment[1];
        rx = highSpeedAdjustment[2];

        setMotorPowers(x, y, rx);

        telemetry.addData("isFloat", isFloat);
    }

    private double calculateMovement(double stickInput, double multiplier) {
        if (Math.abs(stickInput) < STICK_DEADZONE) {
            return multiplier * getLowSpeedFactor(stickInput);
        } else {
            return multiplier * getHighSpeedFactor(stickInput);
        }
    }

    private double getLowSpeedFactor(double stickInput) {
        return (stickInput == gamepad1.left_stick_y) ? Math.abs(stickInput) * 1.67 : Math.abs(stickInput);
    }

    private double getHighSpeedFactor(double stickInput) {
        double v = 0.6 * Math.pow(Math.abs(stickInput), 1.4);
        return (stickInput == gamepad1.left_stick_y) ? 0.33 + v : 0.1412 + v;
    }

    private double calculateRotation(double stickInput, double multiplier) {
        if (Math.abs(stickInput) < STICK_DEADZONE) {
            return multiplier * 0.5 * Math.pow(Math.abs(stickInput), 0.333);
        } else {
            return multiplier * (0.33 + 0.5 * Math.pow(Math.abs(stickInput), 1.4));
        }
    }

    private double[] adjustForHighSpeed(double x, double y, double rx) {
        if (x > 0.85 && rx < 0.2) {
            x *= 1.5;
            y /= 1.5;
        }
        x *= Math.signum(gamepad1.left_stick_y);
        y *= Math.signum(gamepad1.left_stick_x);
        rx *= Math.signum(gamepad1.right_stick_x);

        return new double[]{x, y, rx};
    }

    private void setMotorPowers(double x, double y, double rx) {
        double fl = (-x + y + rx);
        double bl = (-x - y + rx);
        double fr = (-x - y - rx);
        double br = (-x + y - rx);
        robot.setDrivetrainPowers(fl, bl, br, fr);
    }
    //endregion
}
