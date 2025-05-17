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

@Config
@TeleOp(name = "A - EthanOpp")
public class EthanOpp extends EnhancedOpMode {
    // Constants
    private static double STICK_DEADZONE = 0.3;
    private static double TRIGGER_THRESHOLD = 0.1;
    private static double EXTENSION_INCREMENT = 1.0;
    private static double REGULAR_SPEED = 1.0;
    private static double SLOW_SPEED = 0.65;
    private static double SLOW_STRAFE = 0.7;
    private static double PRECISE_SPEED = 0.5;
    private static double PRECISE_STRAFE = 0.6;

    // State variables
    private double extraBasketExtension = 0;
    private boolean intakeTop = false;
    private boolean clawIntake = false;
    private boolean isHighBasket = true;
    private boolean slowExtake = false;
    private boolean firstHangAction = true;
    private boolean firstDropdown = true;

    // Robot control
    private RobotActions actions;
    private ElapsedTime resetTimer;

    // Rumble effect for controller feedback
    private final Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, 200)
            .addStep(0, 0, 200)
            .addStep(1, 1, 200)
            .build();

    @Override
    public void initialize() {
        autoDriveTimer = new ElapsedTime();
        Context.type = DRIVE_TYPE.MANUAL;
        actions = robot.actions;
        robot.pivotExtension.setState(PivotExtension.PivotState.HANG_TOPLOCK);
        robot.pivotExtension.resetTimer.reset();
    }

    @Override
    public void initializeLoop() { }

    @Override
    public void onStart() {
        resetTimer = new ElapsedTime();
        robot.intakeSpecimen.setState(IntakeSpecimen.WristState.SAMPLE_INTAKE);
    }

    @Override
    protected boolean automaticRobotWrite() { return true; }

    @Override
    public void primaryLoop() {
        handleDrivetrainMovement();

//        if (resetTimer.milliseconds() > RESET_DELAY_MS) {
            handleActionMode();
//        }

        robot.tel.addData("actionMode", actionMode.name());
    }

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
        // Reset from hang mode
        if ((CONTROLLER.B1.wasJustPressed() || CONTROLLER.B2.wasJustPressed()) && !firstHangAction) {
            firstDropdown = true;
            actionMode = ACTION_MODE.INTAKE;
            hangSequenceCount = 0;
            robot.runAction(actions.resetFromHang());
        }

        if (firstHangAction) firstHangAction = false;

        // Progress hang sequence
        if ((CONTROLLER.A1.wasJustPressed() || CONTROLLER.A2.wasJustPressed()) && hangSequenceCount <= 1) {
            hangSequenceCount++;
            if (hangSequenceCount == 1) {
                robot.setDrivetrainPowers(0, 0, 0, 0);
                robot.runAction(actions.hangLow());
            }
        }

        // Unhook from bar
        if ((CONTROLLER.RB1.wasJustPressed() || CONTROLLER.RB2.wasJustPressed()) && hangSequenceCount == 2) {
            robot.runAction(actions.unhookFirstBarHang());
            robot.hang.leftPitch.pwm(false);
            robot.hang.rightPitch.pwm(false);
        }

        // Hang movement controls
        handleHangMovement();
        handleHangPitchAdjustment();

        // Normal driving if not in active hang
        if (hangSequenceCount == 0) {
            driveMove();
        }

        robot.tel.addData("hangSequenceCount", hangSequenceCount);
    }

    private void handleHangMovement() {
        if (hangSequenceCount >= 2) {
            double rightPower = -gamepad1.left_stick_y - gamepad2.left_stick_y;
            double leftPower = -gamepad1.right_stick_y - gamepad2.right_stick_y;

            if (CONTROLLER.A1.isDown() || CONTROLLER.A2.isDown()) {
                robot.setDrivetrainPowers(-rightPower, -rightPower, -rightPower, -rightPower);
            } else {
                robot.setDrivetrainPowers(-leftPower, -leftPower, -rightPower, -rightPower);
            }
        }
    }

    private void handleHangPitchAdjustment() {
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD || gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            double adjustment = pitchSpeed * gamepad1.left_trigger;
            Hang.LeftPitchState.INIT.setValue(Hang.LeftPitchState.INIT.getValue() + adjustment);
            Hang.RightPitchState.INIT.setValue(Hang.RightPitchState.INIT.getValue() + adjustment);
        }

        if (gamepad1.right_trigger > TRIGGER_THRESHOLD || gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            double adjustment = pitchSpeed * gamepad1.right_trigger;
            Hang.LeftPitchState.INIT.setValue(Hang.LeftPitchState.INIT.getValue() - adjustment);
            Hang.RightPitchState.INIT.setValue(Hang.RightPitchState.INIT.getValue() - adjustment);
        }
    }

    private void handleIntakeActions() {
        // Toggle intake modes
        handleIntakeToggles();

        // Handle intake operation
        if (CONTROLLER.RB1.isDown()) {
            if (clawIntake) {
                robot.intakeSpecimen.setState(IntakeSpecimen.ClawState.CLOSED);
            } else {
                robot.runAction(actions.justIntake());
            }
        } else if (robot.intakeSpecimen.getState(IntakeSpecimen.PowerState.class).equals(IntakeSpecimen.PowerState.OFF)) {
            robot.intakeSpecimen.setState(IntakeSpecimen.PowerState.HOLDING);
        }

        // Handle dropdown operations
        handleDropdownOperations();

        // Handle extake
        if (CONTROLLER.X1.isDown()) {
            robot.runAction(actions.justExtake());
            ninja = REGULAR_SPEED;
            ninjaStrafe = REGULAR_SPEED;
        }

        // Transitions to other modes
        handleIntakeModeTransitions();

        // Extension toggle
        handleExtensionToggle();

        // Speed controls
        handleSpeedControls();

        // Sweeper controls
        handleSweeperToggle();
    }

    private void handleIntakeToggles() {
        if (CONTROLLER.DPAD_UP1.wasJustPressed()) {
            clawIntake = !clawIntake;
            gamepad1.runRumbleEffect(rumbleEffect);
            gamepad2.runRumbleEffect(rumbleEffect);
        }
//
//        if (CONTROLLER.DPAD_LEFT1.wasJustPressed()) {
//            intakeTop = !intakeTop;
//            gamepad1.runRumbleEffect(rumbleEffect);
//            gamepad2.runRumbleEffect(rumbleEffect);
//        }
    }

    private void handleDropdownOperations() {
        if (CONTROLLER.DPAD_DOWN2.isDown() && CONTROLLER.RT1.isDown()) {
            robot.runAction(actions.justTopDropdown());
        } else if (CONTROLLER.RT1.isDown()) {
            if (firstDropdown) {
                firstDropdown = false;
                if (clawIntake) {
                    robot.runAction(actions.clawDropdown());
                } else {
                    robot.runAction(actions.sweep());
                    if (intakeTop) {
                        robot.runAction(actions.justTopDropdown());
                    } else {
                        robot.runAction(actions.justDropdown());
                    }
                }
            }
        } else if (CONTROLLER.RT1.wasJustReleased()) {
            firstDropdown = true;
            robot.runAction(actions.resetToIntakeHover());
            ninja = REGULAR_SPEED;
            ninjaStrafe = REGULAR_SPEED;
        }
    }

    private void handleIntakeModeTransitions() {
        if (CONTROLLER.LB1.wasJustPressed()) {
            transitionToBasketMode();
        }

        if (CONTROLLER.Y1.wasJustPressed()) {
            robot.runAction(actions.specimenGoToPickup());
            actionMode = ACTION_MODE.SPECIMEN_PICKUP;
        }
    }

    private void transitionToBasketMode() {
        if (isHighBasket) {
            if (clawIntake) {
                robot.runAction(actions.highBasketExtensionClaw(false));
            } else {
                robot.runAction(actions.highBasketExtensionActive(false, extraBasketExtension));
            }
        } else {
            if (clawIntake) {
                robot.runAction(actions.lowBasketExtensionClaw());
            } else {
                robot.runAction(actions.lowBasketExtensionIntake());
            }
        }
        actionMode = ACTION_MODE.BASKET;
        ninja = SLOW_SPEED;
        ninjaStrafe = SLOW_STRAFE;
    }

    private void handleExtensionToggle() {
        if (CONTROLLER.LT1.wasJustPressed()) {
            boolean isReset = robot.pivotExtension.getState(PivotExtension.ExtensionState.class)
                    .equals(PivotExtension.ExtensionState.RESET);
            robot.pivotExtension.setState(isReset ?
                    PivotExtension.ExtensionState.INTAKE_EXTENSION :
                    PivotExtension.ExtensionState.RESET);
        }
    }

    private void handleSpeedControls() {
        if (CONTROLLER.RT2.isDown()) {
            ninja = PRECISE_SPEED;
            ninjaStrafe = PRECISE_STRAFE;
        } else if (CONTROLLER.RT2.wasJustReleased()) {
            ninja = REGULAR_SPEED;
            ninjaStrafe = REGULAR_SPEED;
        }
    }

    private void handleSweeperToggle() {
        if (CONTROLLER.DPAD_RIGHT1.wasJustPressed()) {
            boolean sweepersOut = robot.intakeSpecimen.getState(IntakeSpecimen.LeftSweeperState.class)
                    .equals(IntakeSpecimen.LeftSweeperState.OUT);
            robot.intakeSpecimen.setState(sweepersOut ?
                    IntakeSpecimen.LeftSweeperState.IN :
                    IntakeSpecimen.LeftSweeperState.OUT);
            robot.intakeSpecimen.setState(sweepersOut ?
                    IntakeSpecimen.RightSweeperState.IN :
                    IntakeSpecimen.RightSweeperState.OUT);
        }
    }

    private void handleBasketActions() {
        IntakeSpecimen.PowerState powerState = slowExtake ?
                IntakeSpecimen.PowerState.SLOW_SCORE :
                IntakeSpecimen.PowerState.SCORE;

        // Handle scoring
        if (CONTROLLER.LT1.isDown() || CONTROLLER.RB2.isDown()) {
            robot.intakeSpecimen.setState(powerState);

            if (slowExtake && isHighBasket) {
                // high basket
                PivotExtension.ExtensionState.MANUAL.setValue(
                        PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE.getValue() +
                                extraBasketExtension + 3);
                robot.pivotExtension.setState(PivotExtension.ExtensionState.MANUAL);
            } else if (slowExtake) {
                // low basket
                PivotExtension.ExtensionState.MANUAL.setValue(
                        PivotExtension.ExtensionState.LOW_BASKET_INTAKE.getValue() +
                                extraBasketExtension + 2);
                robot.pivotExtension.setState(PivotExtension.ExtensionState.MANUAL);
            } else if (!isHighBasket){
                robot.pivotExtension.setState(PivotExtension.ExtensionState.LOW_BASKET_INTAKE);
            } else {
                robot.pivotExtension.setState(PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE);
            }

            ninja = SLOW_SPEED;
            ninjaStrafe = SLOW_STRAFE;
        }
        // Handle transitions
        else if (CONTROLLER.LT1.wasJustReleased()) {
            transitionToIntakeFromBasket();
        } else if (CONTROLLER.Y1.wasJustPressed()) {
            transitionToSpecimenFromBasket();
        }

        // Handle extension adjustments
        /* if (robot.pivotExtension.pivotPosition > 95) {
            handleBasketExtensionAdjustment();
        } */
    }

    private void transitionToIntakeFromBasket() {
        robot.runAction(actions.intake(true));
        PivotExtension.ExtensionState.MANUAL.setValue(
                PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE.getValue());
        firstDropdown = true;
        actionMode = ACTION_MODE.INTAKE;
        ninja = REGULAR_SPEED;
        ninjaStrafe = REGULAR_SPEED;
    }

    private void transitionToSpecimenFromBasket() {
        robot.runAction(actions.basketScoreToSpecimenPickup());
        actionMode = ACTION_MODE.SPECIMEN_PICKUP;
        ninja = REGULAR_SPEED;
        ninjaStrafe = REGULAR_SPEED;
    }

    private void handleBasketExtensionAdjustment() {
        if (CONTROLLER.RT1.wasJustPressed()) {
            extraBasketExtension += EXTENSION_INCREMENT;
            updateBasketExtension();
        }
        if (CONTROLLER.RB1.wasJustPressed()) {
            extraBasketExtension -= EXTENSION_INCREMENT;
            updateBasketExtension();
        }
    }

    private void updateBasketExtension() {
        PivotExtension.ExtensionState.MANUAL.setValue(
                PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE.getValue() + extraBasketExtension);
        robot.pivotExtension.setState(PivotExtension.ExtensionState.MANUAL);
    }

    private void handleSpecimenPickupActions() {
        // Reset to intake mode
        if (CONTROLLER.RT1.isDown()) {
            robot.runAction(actions.resetToIntakeHover());
            firstDropdown = true;
            actionMode = ACTION_MODE.INTAKE;
            ninja = SLOW_SPEED;
            ninjaStrafe = SLOW_STRAFE;
        }

        // Slow mode
        if (CONTROLLER.RT1.isDown()) {
            ninja = SLOW_SPEED;
            ninjaStrafe = SLOW_STRAFE;
        }

        // Pickup specimen
        if (CONTROLLER.RB1.wasJustPressed()) {
            gamepad1.runRumbleEffect(rumbleEffect);
            gamepad2.runRumbleEffect(rumbleEffect);
            robot.runAction(actions.specimenPickupTele());
            actionMode = ACTION_MODE.SPECIMEN_DEPOSIT;
        }

        // Return to intake
        if (CONTROLLER.Y1.wasJustPressed()) {
            robot.runAction(actions.specimenToIntake());
            firstDropdown = true;
            actionMode = ACTION_MODE.INTAKE;

        }
    }

    private void handleSpecimenDepositActions() {
        // Score specimen
        if (CONTROLLER.RB1.wasJustPressed()) {
            robot.runAction(actions.specimenScore(true));
            actionMode = ACTION_MODE.SPECIMEN_PICKUP;
        }

        // Return to intake
        if (CONTROLLER.Y1.wasJustPressed()) {
            robot.runAction(actions.specimenToIntake());
            firstDropdown = true;
            actionMode = ACTION_MODE.INTAKE;
        }
    }

    private void handleDrivetrainMovement() {
        // Toggle slow extake
        slowExtake = CONTROLLER.DPAD_LEFT1.wasJustPressed() != slowExtake;

        if (hangSequenceCount == 0) {
            // Transition to hang mode
            if (CONTROLLER.B1.wasJustPressed() || CONTROLLER.B2.wasJustPressed()) {
                robot.hang.isHangActive = true;
                firstHangAction = true;
                actionMode = ACTION_MODE.HANG;
                robot.runAction(actions.hangSetup());
            }

            // Toggle basket height
            if (CONTROLLER.A1.wasJustPressed()) {
                isHighBasket = !isHighBasket;
                gamepad1.runRumbleEffect(rumbleEffect);
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
}
