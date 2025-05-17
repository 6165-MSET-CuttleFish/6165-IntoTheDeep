package org.firstinspires.ftc.teamcode.architecture.modules;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;
import static org.firstinspires.ftc.teamcode.opmodes.ModuleTest.extension;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.control.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.architecture.control.PDFLController;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.modules.moduleUtil.Module;
import org.firstinspires.ftc.teamcode.architecture.modules.moduleUtil.ModuleState;

@Config
public class PivotExtension extends Module {

    public static double inPerTickExtension = 0.0226, friction = 0;
    public static POSITION_CAPS POSITION_CAPS = new POSITION_CAPS();
    public static HANG_PID HANG_PID = new HANG_PID();
    public static MOTION_PROFILE MOTION_PROFILE = new MOTION_PROFILE();
    public static EXTENSION EXTENSION = new EXTENSION();
    public static TELEMETRY TELEMETRY = new TELEMETRY();
    public static INTAKE_SUPPLEMENTAL INTAKE_SUPPLEMENTAL = new INTAKE_SUPPLEMENTAL();
    public static double initExtensionPower = -0.9, initPivotPower = -0.9;
    public static double pivotOffset = 0, pivotOffsetMultiplier = 0.9495, wobbleFactor = 0, extensionOffset = 0;
    public AbsoluteAnalogEncoder elcEncoder;
    public PDFLController pivotController, extensionController;
    public EnhancedMotor pivot, extensionTop, extensionBottom;
    public ElapsedTime motionProfileTimer, resetTimer, extensionProfileTimer;

    public double pivotTarget, pivotPosition, pivotPower, velocity = 0;
    public double extensionDirection = 1, startExtensionTarget = 0, extensionTarget, lastExtensionTarget, extensionProfileTarget, extensionPosition = 0, extensionPower = 0;
    public double lastPivotTarget = pivotTarget + 1, lastPivotPosition = pivotPosition, lastTime = System.nanoTime(), lastTarget = 0;
    public double startPivotTarget, totalDistance = 0, direction = 1;
    public double target = 0, targetVelocity = 0, targetAcceleration = 0, p, d, l, addedPower, velocityAdjustment = 0, accelAdjustment = 0;
    public double rawExtensionPower = 0, rawPivotPower = 0;
    public static double ff = 0;
    public boolean isRawExtensionPower = false, isReset = false, letWristMove = false, isRawPivotPower = false, isInProfile = false, firstWrite = true, profileDone = false;
    HardwareMap hardwareMap;
    double sign = 0;
    double elcFirst = -1;
    int extWriteCount = 0;
    public static class EXTENSION {
        public double p_extension = 0.35, d_extension = 0.01, f_extension = 0.2, l_extension = 0.1, p_hang = 0.7, d_hang = 0, manual_max_velo = 200;
    }

    public static class TELEMETRY {
        public boolean telemetryToggle = true, printCurrent = false, printMotionProfile = false, showPivot = true, showExtension = true;
    }

    public static class POSITION_CAPS {
        public double lowPivotCap = 0, highPivotCap = 185, extensionCap = 46;
    }

    public static class HANG_PID {
        public double pivot_p_hang = 0.38, pivot_d_hang = 0;
    }

    public static class INTAKE_SUPPLEMENTAL {
        public double rightPos = 0.5, initialScale = 0.12;
    }

    public static class MOTION_PROFILE {
        public double p = 0.3, d = 0.027, p_hold = 0.6, d_hold = 0.03;
        public double p_basket1 = 0.3, p_basket2 = 0.3, d_basket1 = 0.0, d_basket2 = 0.0;
        public double pivot_f = -1.4;
        public double holdPIDRange1 = 10, holdPIDRange2 = 6;
        public double kV = 0, kA = 0;
        public double max_accel = 500, max_decel = 600;
        public double inertia = 0;
        public double max_velo = 1000000.1;
    }

    public enum PivotState implements ModuleState {
        INIT(41),
        HANG_TOPLOCK(60),
        LL_READY (10),
        LL_RESET (40),

        FRONT_SPECIMEN_BEFORE (48),
        FRONT_SPECIMEN_AFTER (39),
        INTAKE(0),
        TOP_INTAKE(16),
        CLAW_INTAKE(12),
        BAR_PASS_INTAKE(16),
        BAR_PASS_INTAKE_NON_HOVER(16),
        SPECIMEN_PICKUP(31),
        SPECIMEN_PICKUP_PRELOAD(33),
        SPECIMEN_DEPOSIT_AFTER(70),
        SPECIMEN_PICKUP_EXTENDED(SPECIMEN_PICKUP.value),
        SPECIMEN_RAISE(45),
        SPECIMEN_DEPOSIT(99),
        SPECIMEN_DEPOSIT_PUSH(SPECIMEN_DEPOSIT.value + 10),
        BASKET_POST_DEPOSIT(85),
        HANG_BEFORE(105),
        HANG_PUSHUP(125),
        HANG_AFTER(80),
        HANG_LEVEL_2(0),
        HANG_UNHOOK_AFTER(37),
        HIGH_BASKET_ACTIVE(102),
        HIGH_BASKET_CLAW(102),
        ADA(HIGH_BASKET_ACTIVE.value),
        ADA_LOW(4),
        PARK(55),
        OFF(0);

        private double value;

        PivotState(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = value;
        }
    }

    public enum ExtensionState implements ModuleState {
        RESET(0.02),
        TARTARUS(0),
        UNHOOK(1.2),
        SPECIMEN_OVER(16.5),
        SPECIMEN_UNDER(6.8),
        FRONT_SPECIMEN(18.5),
        SPECIMEN_PICKUP(5),
        HANG_RELEASE(4),
        LOW_BASKET_INTAKE(23),
        LOW_BASKET_CLAW(24),
        HIGH_BASKET_ACTIVE(42),
        HIGH_BASKET_HIGHER(43.5),
        HIGH_BASKET_CLAW(41),
        PRE_HIGH_BASKET_FROM_INTAKE(10),
        PRE_LOW_BASKET_FROM_INTAKE(8),
        HANG_BEFORE(15.5),
        HANG_AFTER(4.25),
        HANG_LEVEL_2(0.1),
        YEET(18),
        BASKET_INTAKE_EXTENSION(9),
        SPECIMEN_PICKUP_EXTENDED(13),
        SPECIMEN_PICKUP_PREEXTENDED(8),
        SHORT_EXTEND(5),
        AUTO_EXTEND(3),
        INTAKE_EXTENSION(21),
        HANG_PREP(11),
        SEMI_INTAKE(RESET.getValue()),
        PARK(14),
        MANUAL(0),

        START_EXTENSION_AUTO(   10),

        SPECIMEN_NEW_DEPOSIT(2);

        private double value;

        ExtensionState(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = value;
        }
    }

    public PivotExtension(HardwareMap hardwareMap) throws InterruptedException {
        super(true, TELEMETRY.telemetryToggle);
        this.hardwareMap = hardwareMap;

        pivot = new EnhancedMotor(hardwareMap, "pivot");
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        extensionTop = new EnhancedMotor(hardwareMap, "extensionTop");
        extensionBottom = new EnhancedMotor(hardwareMap, "extensionBottom");
        extensionTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extensionBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "elc");
        elcEncoder = new AbsoluteAnalogEncoder(encoder);
        elcEncoder.zero(5.6520); // 0.97738
        elcEncoder.setWraparound(false);

        extensionController = new PDFLController();
        pivotController = new PDFLController();

        motionProfileTimer = new ElapsedTime();
        resetTimer = new ElapsedTime();
        extensionProfileTimer = new ElapsedTime();

//        OctoQuad = hardwareMap.get(OctoQuad.class, "OctoQuad");
//        OctoQuad.resetEverything();
//        OctoQuad.setChannelBankConfig(OctoQuadBase.ChannelBankConfig.ALL_PULSE_WIDTH);
//        OctoQuad.setSingleChannelPulseWidthParams (0, new OctoQuad.ChannelPulseWidthParams(1, 1024));
//        OctoQuad.setSingleEncoderDirection(0,  OctoQuadBase.EncoderDirection.FORWARD);
//        OctoQuad.saveParametersToFlash();
    }

    @Override
    protected void write() {

        if (firstWrite) {
            resetTimer.reset();
            isRawExtensionPower = true;
            isReset = true;
            firstWrite = false;
            robot.intakeSpecimen.setState(IntakeSpecimen.WristState.SAMPLE_INTAKE);
        }

        if (!PIVOT_EXTENSION_RESET()) {

//            if ((extWriteCount % 10) == 0) {
//                if (extensionTop.getCurrent(CurrentUnit.AMPS) > 8.5){
//                    robot.opMode.end();
//                }
//            }
//
//            extWriteCount++;
            if (resetTimer.milliseconds() > 300) {
                extensionTop.setPower(extensionPower);
                extensionBottom.setPower(extensionPower);
            }

            if (resetTimer.milliseconds() > 800) {
                pivot.setPower(pivotPower);
            }



        }
    }

    @Override
    public void read() {
        updateExtension();
        updatePivot();
    }

    public void updatePivot() {
        robot.getAccelMagnitude();

        velocity = (pivotPosition - lastPivotPosition) / (System.nanoTime() - lastTime) * Math.pow(10, 9);
        lastPivotPosition = pivotPosition;
        lastTime = System.nanoTime();

        if (Context.read) {
            pivotTarget = Math.max(POSITION_CAPS.lowPivotCap, Math.min(getState(PivotState.class).getValue(), POSITION_CAPS.highPivotCap));
        }

        double startTime = System.nanoTime();
        pivotPosition = (-1 * pivotOffsetMultiplier * pivot.getCurrentPosition() * 360) / 8192 + pivotOffset;

        if (pivotPosition > 110 && extensionPosition > 35) {
            pivotTarget = 110;
        }

        pivotPower = isRawPivotPower ? rawPivotPower : updateMotionProfile();

        // sample intake; specimen pickup; basket / specimen deposit

        /*if (Math.abs(pivotTarget - pivotPosition) > 0.5) {
            if (pivotTarget >= 5 && pivotTarget < 12) {
                friction = 0;
            } else if (pivotTarget >= 12 && pivotTarget < 20) {

                if (pivotTarget < pivotPosition) {
                    friction = 0.05;
                } else {
                    friction = 0.23;
                }
            } else if (pivotTarget >= 25 && pivotTarget < 35) {
                friction = 0.13;
            } else if (pivotTarget >= 80 && pivotTarget < 110) {
                if (extensionTarget > 35) {
                    friction = 0.1;
                } else {
                    friction = 0.03;
                }
            } else {
                friction = 0;
            }
        } else {
            friction = 0;
        }

         */


        if (pivotTarget > 50) {
            friction = 0.05;
            ff = 0.15;
        } else if (pivotTarget > 30) {
            friction = 0.17;
            ff = 0.35;
        } else {
            friction = 0.22;
            ff = 0.42;
        }

        ff *= (0.1781 * Math.sin(0.0425 * extension + 2.414) - 0.255) / (0.1781 * Math.sin(0.0425 * 1 + 2.414) - 0.255);

        pivotPower += friction * Math.signum(pivotTarget - pivotPosition);

        pivotPower += velocityAdjustment + accelAdjustment;


        double intakeDistanceFromSampleSide = robot.intakeSpecimen.wristPosition - INTAKE_SUPPLEMENTAL.rightPos;
        pivotPower += intakeDistanceFromSampleSide * INTAKE_SUPPLEMENTAL.initialScale;

        pivotPower += wobbleFactor * robot.accel;

        if (pivotPosition < 10 && pivotTarget == PivotState.INTAKE.getValue()) pivotPower = 0;

        if (pivotTarget > 95 && pivotPosition > 90) pivotPower = 0.3;


        if (getState(PivotState.class).equals(PivotState.INTAKE) && pivotPosition > PivotState.BAR_PASS_INTAKE.getValue() + 1) {
            pivotTarget = PivotState.BAR_PASS_INTAKE.getValue();
        }
    }

    public void updateExtension() {

        if (Context.read) {
            extensionTarget = getState(ExtensionState.class).getValue() + extensionOffset;
        }

        /*if (getState(ExtensionState.class).equals(ExtensionState.MANUAL)) {
            if (extensionTarget != lastExtensionTarget) {
                extensionProfileTimer.reset();
                lastExtensionTarget = extensionTarget;
                startExtensionTarget = extensionPosition;
                extensionDirection = Math.signum(extensionTarget - extensionPosition);
            }

            if (extensionProfileTimer.seconds() < Math.abs(extensionTarget - startExtensionTarget) / EXTENSION.manual_max_velo) {
                extensionProfileTarget = startExtensionTarget + extensionDirection * extensionProfileTimer.seconds() * EXTENSION.manual_max_velo;
            } else {
                extensionProfileTarget = extensionTarget;
            }
        } else {
            extensionProfileTarget = extensionTarget;
        }

         */


        extensionProfileTarget = extensionTarget;

        extensionPosition = extensionTop.getCurrentPosition() * inPerTickExtension + (pivotPosition / 90 * 0.61);
        extensionProfileTarget = Math.min(extensionProfileTarget, POSITION_CAPS.extensionCap);

        extensionController.update(extensionProfileTarget, extensionPosition);

        if (robot.hang.isHangActive) {
            extensionController.setController(EXTENSION.p_hang, EXTENSION.d_hang, 0, EXTENSION.l_extension);
        } else if (pivotPosition < 20) {
            extensionController.setController(0.2, EXTENSION.d_extension, EXTENSION.f_extension * Math.sin(Math.toRadians(pivotPosition)), EXTENSION.l_extension);
        } else {
            extensionController.setController(EXTENSION.p_extension, EXTENSION.d_extension, EXTENSION.f_extension * Math.sin(Math.toRadians(pivotPosition)), EXTENSION.l_extension);
        }

        extensionPower = isRawExtensionPower ? rawExtensionPower : extensionController.getPDFL();
    }

    public double updateMotionProfile() {
        l = 0;
        /*ff = regressionFeedforward(extensionPosition, pivotPosition) * MOTION_PROFILE.pivot_f;
        ff += -targetAcceleration * MOTION_PROFILE.inertia;

         */

//        if (pivotPosition > 80) ff = 0;

        if (pivotTarget != lastPivotTarget) {
            motionProfileTimer.reset();
            lastPivotTarget = pivotTarget;
            startPivotTarget = pivotPosition;
            totalDistance = pivotTarget - startPivotTarget;
            direction = Math.signum(totalDistance);
            isInProfile = true;
        }

        double targetAccel = MOTION_PROFILE.max_accel;
        double decel = MOTION_PROFILE.max_decel;
        double profileOutput = motion_profile(targetAccel, decel, MOTION_PROFILE.max_velo, Math.abs(totalDistance), motionProfileTimer.seconds());
        target = startPivotTarget + (profileOutput * direction);

        if (robot != null && robot.hang.isHangActive && getState(PivotState.class).equals(PivotState.HANG_AFTER)) {
            p = HANG_PID.pivot_p_hang;
            d = HANG_PID.pivot_d_hang;
        } else if ((getState(PivotState.class).equals(PivotState.HIGH_BASKET_ACTIVE) || getState(PivotState.class).equals(PivotState.HIGH_BASKET_CLAW)) && Math.abs(pivotPosition - pivotTarget) < MOTION_PROFILE.holdPIDRange1) {
            if (Math.abs(pivotPosition - pivotTarget) < MOTION_PROFILE.holdPIDRange2) {
                p = MOTION_PROFILE.p_basket2;
                d = MOTION_PROFILE.d_basket2;
            } else {
                p = MOTION_PROFILE.p_basket1;
                d = MOTION_PROFILE.d_basket1;
            }
        } else {

            if (Math.abs(pivotTarget - pivotPosition) < 1) {
                p = MOTION_PROFILE.p_hold;
                d = MOTION_PROFILE.d_hold;
            } else {
                p = MOTION_PROFILE.p;
                d = MOTION_PROFILE.d;
            }

        }

        double extensionScalar = Math.abs(0.1781 * Math.sin(0.0425 * extensionPosition + 2.414) - 0.255);

        p *= extensionScalar;
        d *= extensionScalar;
//
        pivotController.setController(p /* * 15 / robot.getAvgLooptime()*/, d, ff, 0); // this l is to profile target, not pivotTarget, so we don't want to use it. we made our own l that goes to pivot target

        accelAdjustment = targetAcceleration * MOTION_PROFILE.kA;
        velocityAdjustment = targetVelocity * MOTION_PROFILE.kV;
        lastTarget = target;

        pivotController.update(target, pivotPosition);


        if (Math.abs(totalDistance) > 45 && direction * pivotPosition < direction * (startPivotTarget + totalDistance * MOTION_PROFILE.max_decel/(MOTION_PROFILE.max_decel + MOTION_PROFILE.max_accel))) {
            return direction;
        }

        return pivotController.getPDFL();
    }

    public double motion_profile(double accel, double decel, double max_velocity, double distance, double time) {
        // Calculate the time it takes to accelerate to max velocity
        double accelTime = max_velocity / accel;
        double decelTime = max_velocity / decel;
        // Calculate distances for acceleration and deceleration
        double accelDist = 0.5 * accel * Math.pow(accelTime, 2);
        double decelDist = 0.5 * decel * Math.pow(decelTime, 2);
        double total_min_distance = accelDist + decelDist;

        if (total_min_distance > distance) {
            double ratio = accel / decel;
            double denom = accel + (Math.pow(accel, 2) / decel);
            accelTime = Math.sqrt((2 * distance) / denom);
            decelTime = ratio * accelTime;
            max_velocity = accel * accelTime;

            accelDist = 0.5 * accel * Math.pow(accelTime, 2);
            decelDist = 0.5 * decel * Math.pow(decelTime, 2);
        }

        // Calculate cruise parameters
        double cruise_distance = distance - accelDist - decelDist;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_start = accelTime + cruise_dt;
        double total_time = accelTime + cruise_dt + decelTime;

        if (time >= total_time) {
            targetVelocity = 0;
            targetAcceleration = 0;
            profileDone = true;
            return distance;
        }

        if (time < accelTime) {
            targetAcceleration = accel;
            sign = Math.signum(pivotTarget - pivotPosition);
            targetVelocity = time * accel * sign;
            profileDone = false;
            return 0.5 * accel * Math.pow(time, 2);
        } else if (time < deceleration_start) {
            targetAcceleration = 0;
            targetVelocity = max_velocity * sign;
            double cruise_current_dt = time - accelTime;
            return accelDist + max_velocity * cruise_current_dt;
        } else {
            double decel_current_dt = time - deceleration_start;
            targetAcceleration = -decel;
            targetVelocity = (max_velocity - (decel * decel_current_dt)) * sign;
            return distance - (0.5 * decel * Math.pow(decelTime - decel_current_dt, 2));
        }
    }

    public double regressionFeedforward(double extension, double angle) {
        // y = -0.0000398325x^2 - 0.00387123x + 0.71658
        double pivotScalar = -0.0000398325 * Math.pow(angle, 2) - 0.00387123 * angle + 0.71658;
        double extensionScalar = 0.1781 * Math.sin(0.0425 * extension + 2.414) - 0.255;
        return pivotScalar * extensionScalar;
    }

    @Override
    public void telemetryUpdate() {
        if (TELEMETRY.telemetryToggle) {
            super.telemetryUpdate();
            if (TELEMETRY.showPivot) {
                robot.tel.addData("Start Pivot Target", startPivotTarget);
                robot.tel.addData("Pivot Target", pivotTarget);
                robot.tel.addData("Pivot position state", getState(PivotState.class));
                robot.tel.addData("pivotPower", pivot.getPower());
                robot.tel.addData("pivotPos", pivotPosition);
                robot.tel.addData("pivotTarget", pivotTarget);
                robot.tel.addData("raw rev pos", pivot.getCurrentPosition());
                robot.tel.addData("UnOffset rev pos", (-1 * pivotOffsetMultiplier * pivot.getCurrentPosition() * 360) / 8192);
                robot.tel.addData("total distance", totalDistance);
                robot.tel.addData("isProfileDone", profileDone);
                robot.tel.addData("elc pos", elcEncoder.getCurrentPosition());
            }

            if (TELEMETRY.showExtension) {
                robot.tel.addData("Extension position state", getState(ExtensionState.class));
                robot.tel.addData("extensionPower", extensionTop.getPower());
                robot.tel.addData("extensionPos", extensionPosition);
                robot.tel.addData("extensionTarget", extensionTarget);
                robot.tel.addData("extensionProfileTarget", extensionProfileTarget);
                robot.tel.addData("timer", motionProfileTimer.seconds());
            }

            if (TELEMETRY.printCurrent) {
                robot.tel.addData("pivotCurrent", pivot.getCurrent(CurrentUnit.AMPS));
                robot.tel.addData("extensionCurrent", extensionTop.getCurrent(CurrentUnit.AMPS) + extensionBottom.getCurrent(CurrentUnit.AMPS));
            }

            if (TELEMETRY.printMotionProfile) {
                robot.tel.addData("motionProfileComponent", target - startPivotTarget);
                robot.tel.addData("profileTarget", target);
                robot.tel.addData("targetVelocity", targetVelocity);
                robot.tel.addData("targetAcceleration", targetAcceleration);
                robot.tel.addData("p", p);
                robot.tel.addData("d", d);
                robot.tel.addData("addedPower", addedPower);
                robot.tel.addData("feedforward", ff);
                robot.tel.addData("velocity", velocity);
                robot.tel.addData("drivetrain accel", robot.accel);
                robot.tel.addData("wobble power", wobbleFactor * robot.accel);
            }
        }
    }

    @Override
    protected void initInternalStates() {
        setInternalStates(PivotState.INIT, ExtensionState.RESET);
    }

    @Override
    protected void updateInternalStatus() {
        if (Math.abs(pivotTarget - pivotPosition) < 5 && Math.abs(extensionTarget - extensionPosition) < 3) { // degrees
            status = Status.IDLE;
        } else {
            status = Status.BUSY;
        }
    }

    private boolean PIVOT_EXTENSION_RESET() {
        if (!isReset) return false;

        if (Context.PIVOT_EXTENSION_RESET.hasReset) {
            letWristMove = true;
            pivotOffset = Context.PIVOT_EXTENSION_RESET.pivotOffset;
            isReset = false;
            rawExtensionPower = 0;
            isRawExtensionPower = false;
            return false;
        }
        if (resetTimer.milliseconds() > Context.RESET_DELAY_MS) {
            rawExtensionPower = 0;
            isRawExtensionPower = false;
            robot.intakeSpecimen.setState(IntakeSpecimen.WristState.INIT);
            isReset = false;
            Context.PIVOT_EXTENSION_RESET.hasReset = true;

        } else if (resetTimer.milliseconds() > Context.RESET_DELAY_MS - 100) {
            extensionTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extensionTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (resetTimer.milliseconds() > 300) {
            extensionTop.setPower(initExtensionPower);
            extensionBottom.setPower(initExtensionPower);
            letWristMove = true;
        } else if (resetTimer.milliseconds() > 100) {
            if (elcFirst == -1) {
                elcFirst = elcEncoder.getCurrentPosition();
                RobotLog.e("ELC VALUE: " + elcFirst);
            }
            pivotOffset = pivotOffsetMultiplier * pivot.getCurrentPosition() * 360 / 8192 + elcFirst;
            Context.PIVOT_EXTENSION_RESET.pivotOffset = pivotOffset;
        }
        return true;
    }
}
