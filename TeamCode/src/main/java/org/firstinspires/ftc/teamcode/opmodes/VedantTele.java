/*
package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.architecture.Context.currentIntakeThresh;
import static org.firstinspires.ftc.teamcode.architecture.Context.depositDriveTime;
import static org.firstinspires.ftc.teamcode.architecture.Context.idleHangPower;
import static org.firstinspires.ftc.teamcode.architecture.Context.intakeDriveTime;
import static org.firstinspires.ftc.teamcode.architecture.Context.isFloat;
import static org.firstinspires.ftc.teamcode.architecture.Context.ninja;
import static org.firstinspires.ftc.teamcode.architecture.Context.ninjaStrafe;
import static org.firstinspires.ftc.teamcode.architecture.Context.pitchSpeed;
import static org.firstinspires.ftc.teamcode.architecture.Context.pivotSpeed;
import static org.firstinspires.ftc.teamcode.architecture.Context.wristSpeed;
import static org.firstinspires.ftc.teamcode.architecture.Robot.CONTROLLER;
import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.RobotActions;
import org.firstinspires.ftc.teamcode.architecture.modules.Hang;
import org.firstinspires.ftc.teamcode.architecture.modules.IntakeSpecimen;
import org.firstinspires.ftc.teamcode.architecture.modules.PivotExtension;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.tele.ACTION_MODE;
import org.firstinspires.ftc.teamcode.architecture.tele.DRIVE_TYPE;
import org.firstinspires.ftc.teamcode.vision.Pipelines.IntakePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@TeleOp(name = "C - AntiEthanTele")
public class VedantTele extends EnhancedOpMode {
    public ACTION_MODE actionMode = ACTION_MODE.INTAKE;
    public DRIVE_TYPE type;
    public ElapsedTime autoDriveTimer;
    public int countHangSequence = 0;
    OpenCvWebcam webcam;
    IntakePipeline pipeline;
    RobotActions actions;
    ElapsedTime timer;

    boolean lastIsFloat, linkPower;

    @Override
    public void initialize() {
        autoDriveTimer = new ElapsedTime();
        type = DRIVE_TYPE.MANUAL;
        actions = robot.actions;
        robot.pivotExtension.setState(PivotExtension.PivotState.INIT);
    }

    @Override
    public void initializeLoop() {
        // robot.read();
        // robot.write();
    }

    @Override
    public void onStart() {
        timer = new ElapsedTime();
    }

    @Override
    protected boolean automaticRobotWrite() {
        return true;
    }

    @Override
    public void primaryLoop() {
        DRIVETRAIN_MOVEMENT();

        if (timer.milliseconds() > 1200) {
            */
/**
             * switch cases are literally just fancy if statements
             * e.g. if (actionMode == EXTENSION_RESET) else if (actionMode == HANG) does the same thing
             *//*

            switch (actionMode) {
                case HANG:
                    HANG_ACTIONS();
                    break;
                case INTAKE:
                    INTAKE_ACTIONS();
                    break;
                case BASKET:
                    BASKET_ACTIONS();
                    break;
                case SPECIMEN_PICKUP:
                    SPECIMEN_PICKUP_ACTIONS();
                    break;
                case SPECIMEN_DEPOSIT:
                    SPECIMEN_DEPOSIT_ACTIONS();
                    break;
            }
        }

        robot.tel.addData("actionMode", actionMode.name());
    }

    public void HANG_ACTIONS() {
        linkPower = CONTROLLER.A2.isDown();
        // second part to sequence

        if (Math.abs(gamepad2.left_trigger) > 0.1) {
            robot.pivotExtension.getState(PivotExtension.PivotState.class).setValue(robot.pivotExtension.getState(PivotExtension.PivotState.class).getValue() + pivotSpeed * gamepad2.left_trigger);
        }
        // left pitch

        if (Math.abs(gamepad2.right_trigger) > 0.1) {
            robot.pivotExtension.getState(PivotExtension.PivotState.class).setValue(robot.pivotExtension.getState(PivotExtension.PivotState.class).getValue() - pivotSpeed * gamepad2.right_trigger);
        }
        // right pitch
        if (CONTROLLER.A2.wasJustPressed()) {
            countHangSequence++;
            // sets sequence up to step 1
            if (countHangSequence == 1) {
                robot.runAction(actions.hangLow());
                robot.setDrivetrainPowers(1, 1, 1, 1);
                //
            } else if (countHangSequence == 2) {
                robot.pivotExtension.setState(PivotExtension.ExtensionState.HANG_RELEASE);

                // swing
            } else if (countHangSequence == 3) {
                robot.hang.leftPitch.pwm(false);
                robot.hang.rightPitch.pwm(false);
                robot.runAction(actions.resetHang());
            }
        }

        double leftPower = 0, rightPower = 0;
        double right = -gamepad2.left_stick_y;
        double left = -gamepad2.right_stick_y;
        //hang

        if (countHangSequence >= 1) {
            if (linkPower) {
                leftPower = Math.abs(left) > 0.1 ? left : (countHangSequence >= 2 ? idleHangPower : 0);
                robot.setDrivetrainPowers(-leftPower, -leftPower, -leftPower, -leftPower);
            } else {
                leftPower = Math.abs(left) > 0.1 ? left : (countHangSequence >= 2 ? idleHangPower : 0);
                rightPower = Math.abs(right) > 0.1 ? right : (countHangSequence >= 2 ? idleHangPower : 0);
                robot.setDrivetrainPowers(-leftPower, -leftPower, -rightPower, -rightPower);
            }
        }

        robot.tel.addData("countHangSequence-------", countHangSequence);
        robot.tel.addData("rightPower-----", rightPower);
        robot.tel.addData("leftPower-----", leftPower);

        robot.hang.setState(Hang.LeftPitchState.MOVING);

        if (gamepad2.left_trigger > 0.1) {
            Hang.LeftPitchState.MOVING.setValue(Hang.LeftPitchState.MOVING.getValue() + pitchSpeed * gamepad2.left_trigger);
        }
        if (gamepad2.right_trigger > 0.1) {
            Hang.LeftPitchState.MOVING.setValue(Hang.LeftPitchState.MOVING.getValue() - pitchSpeed * gamepad2.right_trigger);
        }

        if (countHangSequence == 0) {
            driveMove();
        }
    }

    public void INTAKE_ACTIONS() {
        if (robot.intakeSpecimen.intake.getCurrent(CurrentUnit.AMPS) > currentIntakeThresh) {
            gamepad1.rumble(300);
            gamepad2.rumble(300);
        }

        if (CONTROLLER.RB2.isDown()) {
            robot.runAction(actions.justIntake());
            // intake
        } else if (CONTROLLER.RB2.wasJustReleased()) {
            robot.intakeSpecimen.setState(IntakeSpecimen.PowerState.OFF);
        } //speci

        if (CONTROLLER.LB2.isDown() && CONTROLLER.RT2.isDown()) {
            robot.runAction(actions.justDropdown());
            if (gamepad2.left_trigger > 0.1) {
                IntakeSpecimen.WristState.SAMPLE_INTAKE.setValue(IntakeSpecimen.WristState.SAMPLE_INTAKE.getValue() + wristSpeed * gamepad2.left_trigger);
            }
        } else if (CONTROLLER.RT2.isDown()) {
            robot.runAction(actions.justDropdown());
        } else if (CONTROLLER.RT2.wasJustReleased()) {
            robot.runAction(actions.resetToIntakeHover());
            ninja = 1;
            ninjaStrafe = 1;
        }

        if (CONTROLLER.LT1.isDown()) {
            robot.runAction(actions.justExtake());
            ninja = 1;
            ninjaStrafe = 1;
        }

        if (CONTROLLER.LB2.wasJustPressed()) {
            robot.runAction(actions.highBasketExtension(true));
            actionMode = ACTION_MODE.BASKET;
            ninja = 0.5;
            ninjaStrafe = 0.6;
        }

        if (CONTROLLER.LT2.wasJustPressed()) {
            robot.runAction(actions.specimenScore());
            actionMode = ACTION_MODE.SPECIMEN_PICKUP;
        }

        if (CONTROLLER.B2.isDown()) {
            ninja = 0.6;
            ninjaStrafe = 8;
        } else if (CONTROLLER.B2.wasJustReleased()) {
            ninja = 0.35;
            ninjaStrafe = 0.5;
        }

        if (CONTROLLER.X2.wasJustPressed()) {
            if (robot.intakeSpecimen.getState(IntakeSpecimen.SweeperState.class).equals(IntakeSpecimen.SweeperState.IN)) {
                robot.intakeSpecimen.setState(IntakeSpecimen.SweeperState.OUT);
            } else {
                robot.intakeSpecimen.setState(IntakeSpecimen.SweeperState.IN);
            }
        }
    }

    public void BASKET_ACTIONS() {
        IntakeSpecimen.PowerState powerState = CONTROLLER.Y2.isDown() ? IntakeSpecimen.PowerState.EGSCORE : IntakeSpecimen.PowerState.SCORE;

        if (CONTROLLER.X2.isDown()) {
            robot.intakeSpecimen.setState(powerState);
            ninja = 0.5;
            ninjaStrafe = 0.7;
        } else if (CONTROLLER.RT2.wasJustReleased()) {
            robot.runAction(actions.intake(true));
            actionMode = ACTION_MODE.INTAKE;
            ninja = 1;
            ninjaStrafe = 1;
        } else if (CONTROLLER.Y2.wasJustReleased()) {
            robot.runAction(actions.basketScoreToSpecimenPickup());
            actionMode = ACTION_MODE.SPECIMEN_PICKUP;
            ninja = 1;
            ninjaStrafe = 1;
        }
    }

    public void SPECIMEN_PICKUP_ACTIONS() {
        if (CONTROLLER.RT2.isDown()) {
            robot.runAction(actions.resetToIntakeHover());
            actionMode = ACTION_MODE.INTAKE;
            ninja = 0.5;
            ninjaStrafe = 0.75;
        }

        if (CONTROLLER.Y2.isDown()) {
            ninja = 0.5;
            ninjaStrafe = 0.75;
        }

        if (CONTROLLER.LB2.wasJustPressed()) {
            robot.runAction(actions.specimenPickup());
            actionMode = ACTION_MODE.SPECIMEN_DEPOSIT;
        }
    }

    public void SPECIMEN_DEPOSIT_ACTIONS() {
        if (CONTROLLER.LB2.wasJustPressed()) {
            robot.runAction(actions.specimenScore());
            actionMode = ACTION_MODE.SPECIMEN_PICKUP;
        }
    }

    public void DRIVETRAIN_MOVEMENT() {
        if (!actionMode.equals(ACTION_MODE.HANG)) {
            if (CONTROLLER.DPAD_RIGHT2.wasJustPressed()) {
                robot.hang.setHangActive(true);
                actionMode = ACTION_MODE.HANG;
                robot.runAction(actions.hangSetup());
                robot.hang.isHangActive = true;
            }

            switch (type) {
                case AUTO_BASKET:
                    handleAutoDrive(-1);
                    break;
                case AUTO_SUBMERSIBLE:
                    handleAutoDrive(1);
                    break;
                default:
                    driveMove();
                    break;
            }
        }
    }

    private void handleAutoDrive(int power) {
        robot.setDrivetrainPowers(power, power, power, power);

        double driveTimeLimit = (power == -1) ? intakeDriveTime : depositDriveTime;
        if (autoDriveTimer.milliseconds() > driveTimeLimit) {
            type = DRIVE_TYPE.MANUAL;
        }
    }

    public void LIMELIGHT_PRINT() {
        Pose2d samplePos = robot.samplePos(robot.pose, robot.limelight.getTarget());
        robot.tel.addData("samplePos", "(" + samplePos.position.x + "," + samplePos.position.y + ")");
    }

    public void driveMove() {
        double x = calculateMovement(gamepad2.left_stick_y, ninja);
        double y = calculateMovement(gamepad2.left_stick_x, ninjaStrafe);
        double rx = calculateRotation(gamepad2.right_stick_x, ninja);

        double[] highSpeedAdjustment = adjustForHighSpeed(x, y, rx);
        x = highSpeedAdjustment[0];
        y = highSpeedAdjustment[1];
        rx = highSpeedAdjustment[2];

        setMotorPowers(x, y, rx);

        if (!isFloat && lastIsFloat) {
            setZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (isFloat && !lastIsFloat) {
            setZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        lastIsFloat = isFloat;
        telemetry.addData("isFloat", isFloat);
    }

    public void setZeroPower(DcMotor.ZeroPowerBehavior p) {
        */
/*
        robot.fl.setZeroPowerBehavior(p);
        robot.bl.setZeroPowerBehavior(p);
        robot.fr.setZeroPowerBehavior(p);
        robot.br.setZeroPowerBehavior(p);
        *//*

    }

    private double calculateMovement(double stickInput, double multiplier) {
        if (Math.abs(stickInput) < 0.3) {
            return multiplier * getLowSpeedFactor(stickInput);
        } else {
            return multiplier * getHighSpeedFactor(stickInput);
        }
    }

    private double getLowSpeedFactor(double stickInput) {
        return (stickInput == gamepad2.left_stick_y) ? Math.abs(stickInput) * 1.67 : Math.abs(stickInput);
    }

    private double getHighSpeedFactor(double stickInput) {
        return (stickInput == gamepad2.left_stick_y) ? 0.33 + 0.6 * Math.pow(Math.abs(stickInput), 1.4) : 0.1412 + 0.6 * Math.pow(Math.abs(stickInput), 1.4);
    }

    private double calculateRotation(double stickInput, double multiplier) {
        if (Math.abs(stickInput) < 0.3) {
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
        x *= Math.signum(gamepad2.left_stick_y);
        y *= Math.signum(gamepad2.left_stick_x);
        rx *= Math.signum(gamepad2.right_stick_x);

        return new double[]{x, y, rx};
    }

    private void setMotorPowers(double x, double y, double rx) {
        double fl = (-x + y + rx);
        double bl = (-x - y + rx);
        double fr = (-x - y - rx);
        double br = (-x + y - rx);
        robot.setDrivetrainPowers(fl, bl, br, fr);
    }

    public void visionInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("intakeCamera", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "intakeCamera"), cameraMonitorViewId);
        pipeline = new IntakePipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("error", errorCode);
                telemetry.update();
            }
        });
    }
}
*/
