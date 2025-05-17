package org.firstinspires.ftc.teamcode.architecture;

import static org.firstinspires.ftc.teamcode.architecture.Context.depositWaitTime;
import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;
import static org.firstinspires.ftc.teamcode.architecture.modules.Limelight.PAST_DISTANCE;
import static org.firstinspires.ftc.teamcode.architecture.modules.Limelight.PRE_DISTANCE;
import static org.firstinspires.ftc.teamcode.architecture.modules.Limelight.isDoneIntakeAction;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.architecture.modules.Hang;
import org.firstinspires.ftc.teamcode.architecture.modules.IntakeSpecimen;
import org.firstinspires.ftc.teamcode.architecture.modules.Limelight;
import org.firstinspires.ftc.teamcode.architecture.modules.PivotExtension;
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Builder;
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Task;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * IMPORTANT: RobotActions HAS NO RUN CAPABILITIES
 * all it is is a store of task lists, all the running happens in task scheduler
 * I know it's hard but if yall can, can you try to break them up into smaller lists and build them up?
 * Often two macros use the same sub-sequence and we have to go change both
 * Also if anyone has ideas to organize actions better, pls do
 */
@Config
public class RobotActions {

    public List<Task> unhookFirstBarHang() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.HANG_RELEASE)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HANG_UNHOOK_AFTER)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .delay(500)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .build();
    }

    public List<Task> limelightIntake(double extend_amt) {
        ElapsedTime timer = new ElapsedTime();

        return Builder.create()
               // .executeCode(() -> )
//                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)

                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.LL_READY)
                .executeCode(() -> PivotExtension.ExtensionState.MANUAL.setValue(extend_amt - PRE_DISTANCE))
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.MANUAL)
                .await(() -> (Math.abs(robot.pivotExtension.extensionPosition - robot.pivotExtension.extensionTarget) < 2 && (Math.abs(robot.pivotExtension.pivotPosition - robot.pivotExtension.pivotTarget) < 10)) || timer.milliseconds() > 1500)
                .delay(500)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)
                .await(() -> robot.pivotExtension.pivotPosition < 10)

                .executeCode(() -> RobotLog.e("VALUEEEE" + PivotExtension.ExtensionState.MANUAL.getValue()))

                .executeCode(timer::reset)

                .executeCode(() -> PivotExtension.ExtensionState.MANUAL.setValue(extend_amt + PAST_DISTANCE))
                .await(() -> Math.abs(robot.pivotExtension.extensionPosition - robot.pivotExtension.extensionTarget) < 1)
                .delay(1000)
                .executeCode(() -> isDoneIntakeAction = true)
                .build();
    }

    public List<Task> limelightDestroyBlockInTheWayIntake(double extend_amt) {
        ElapsedTime timer = new ElapsedTime();

        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE)
                .executeCode(() -> PivotExtension.ExtensionState.MANUAL.setValue(extend_amt - PRE_DISTANCE))
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.MANUAL)

                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_PICKUP)

                .await(() -> (Math.abs(robot.pivotExtension.extensionPosition - robot.pivotExtension.extensionTarget) < 1 && (Math.abs(robot.pivotExtension.pivotPosition - robot.pivotExtension.pivotTarget) < 3)) || timer.milliseconds() > 1000)

                .delay(400)

                .executeCode(() -> PivotExtension.ExtensionState.MANUAL.setValue(extend_amt+0.5))

                .await(() -> (Math.abs(robot.pivotExtension.extensionPosition - robot.pivotExtension.extensionTarget) < 2 || timer.milliseconds() > 1000))

                .delay(200)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .executeCode(() -> PivotExtension.ExtensionState.MANUAL.setValue(extend_amt-1))

                .await(() -> (Math.abs(robot.pivotExtension.extensionPosition - robot.pivotExtension.extensionTarget) < 2 || timer.milliseconds() > 1000))

                .delay(200)



                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .delay(100)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)

                .await(() -> ((Math.abs(robot.pivotExtension.pivotPosition - robot.pivotExtension.pivotTarget) < 3)) || timer.milliseconds() > 500)

                .delay(200)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)

                .executeCode(() -> PivotExtension.ExtensionState.MANUAL.setValue(extend_amt + PAST_DISTANCE))

                .executeCode(timer::reset)

                .await(() -> Math.abs(robot.pivotExtension.extensionPosition - robot.pivotExtension.extensionTarget) < 1)
                .delay(1000)
                .executeCode(() -> isDoneIntakeAction = true)
                .build();
    }

    public List<Task> lookaheadIntake(double extend_amt) {
        ElapsedTime timer = new ElapsedTime();

        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)
                .await(() -> robot.pivotExtension.pivotPosition < 10)

                .executeCode(() -> RobotLog.e("VALUEEEE" + PivotExtension.ExtensionState.MANUAL.getValue()))

                .executeCode(timer::reset)

                .executeCode(() -> PivotExtension.ExtensionState.MANUAL.setValue(extend_amt + PRE_DISTANCE + PAST_DISTANCE))
                .await(() -> Math.abs(robot.pivotExtension.extensionPosition - robot.pivotExtension.extensionTarget) < 1)
                .delay(1000)
                .executeCode(() -> isDoneIntakeAction = true)
                .build();
    }

    public List<Task> specimenNewScore() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.SPECIMEN_DEPOSIT)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_TOP_INTAKE)

                .delay(200)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.SPECIMEN_NEW_DEPOSIT)

//                .await(() -> robot.pivotExtension.extensionPosition + 5.5 > PivotExtension.ExtensionState.SPECIMEN_OVER.getValue())
                .delay(200)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_PICKUP)
                .delay(600)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.SPECIMEN_PICKUP)


                .delay(1000)


                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_PICKUP)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)

                // .await(() -> robot.pivotExtension.extensionPosition < Context.extensionWaitToPivotIntake)
                .build();
        // .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.OPEN)

    }

    public List<Task> specimenNewPickupTele() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.YEET)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .delay(200)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.SPECIMEN_RAISE)
                .delay(100)
                .delay(500)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.SPECIMEN_DEPOSIT)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .delay(200)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.SPECIMEN_NEW_DEPOSIT)
                .delay(150)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_DEPOSIT)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .build();
    }

    public List<Task> extendAndIntake(double preVal, double postVal) {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.MANUAL)
                .executeCode(() -> PivotExtension.ExtensionState.MANUAL.setValue(preVal))
                .delay(500)
                .executeCode(() -> PivotExtension.ExtensionState.MANUAL.setValue(postVal))
                .build();
    }

//    public List<Task> hangRelease() {
//        return Builder.create()
//                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.HANG_RELEASE)
//                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HANG_LEVEL_3)
//                .await(() -> robot.pivotExtension.pivotPosition < PivotExtension.PivotState.HANG_LEVEL_3.getValue() + 5)
//                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.OFF)
//                .build();
//    }

    public List<Task> ActivateIntake() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                .executeCode(() -> RobotLog.e("intake"))
                .build();
    }

    public List<Task> firstExtendAndActivateIntake() {
        return Builder.create()
//                .await(()-> robot.pivotExtension.getState(PivotExtension.ExtensionState.class).equals(PivotExtension.ExtensionState.RESET))
//                .await(() -> Math.abs(robot.pivotExtension.extensionPosition - robot.pivotExtension.extensionTarget) < 5)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .delay(500)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)
                .delay(200)


                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                .executeCode(() -> RobotLog.e("intake"))
                .build();
    }

    public List<Task> extendAndActivateIntakePark() {
        return Builder.create()
//                .await(()-> robot.pivotExtension.getState(PivotExtension.ExtensionState.class).equals(PivotExtension.ExtensionState.RESET))
//                .await(() -> Math.abs(robot.pivotExtension.extensionPosition - robot.pivotExtension.extensionTarget) < 5)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.PARK)
                .delay(200)
//                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.INTAKE_EXTENSION)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                .executeCode(() -> RobotLog.e("intake"))
                .build();
    }

    public List<Task> submersibleIntake() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.YEET)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                .executeCode(() -> RobotLog.e("intake"))
                .build();
    }

    public List<Task> activateExtake() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.EXTAKE)
                .executeCode(() -> RobotLog.e("extake"))
                .build();
    }

    public List<Task> activateIntake(double extendLength) {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)

                .executeCode(() -> PivotExtension.ExtensionState.MANUAL.setValue(extendLength))
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.MANUAL)
                .executeCode(() -> RobotLog.e("extake"))
                .build();
    }


    public List<Task> extendAndActivateExtake(long activeIntakeDelayMs) {
        return Builder.create()

                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.YEET)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .delay(400)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.YEET)
                .delay(200)
                .addTaskList(robot.actions.autoIntakeReset())
                .delay(activeIntakeDelayMs)
                .addTaskList(ActivateIntake())
                .executeCode(() -> RobotLog.e("intake"))
                .build();
    }

    public List<Task> readyIntake(PivotExtension.ExtensionState extensionState) {
        return Builder.create()
                .moduleAction(robot.pivotExtension, extensionState)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .build();
    }

    public List<Task> extakeCycleAndExtend(int delayMs, double extendLength) {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.SLOW_EXTAKE)
                .delay(delayMs)
                .addTaskList(activateIntake(extendLength))
                .executeCode(() -> PivotExtension.ExtensionState.MANUAL.setValue(extendLength))
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.MANUAL)
                .build();
    }

    //only dropdown (no intake)
    public List<Task> justDropdown() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)
                .await(() -> robot.pivotExtension.pivotPosition < PivotExtension.PivotState.BAR_PASS_INTAKE.getValue() + 10)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.INTAKE_EXTENSION)
//                .delay(400)
//                .moduleAction(robot.pivotExtension, robot.intakeSpecimen.getState(IntakeSpecimen.WristState.class).equals(IntakeSpecimen.WristState.SAMPLE_INTAKE) ?
//                        PivotExtension.ExtensionState.INTAKE1 : robot.pivotExtension.getState(PivotExtension.ExtensionState.class))
                .build();
    }
    public List<Task> justRetract() {
        return Builder.create()

                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
//                .delay(400)
//                .moduleAction(robot.pivotExtension, robot.intakeSpecimen.getState(IntakeSpecimen.WristState.class).equals(IntakeSpecimen.WristState.SAMPLE_INTAKE) ?
//                        PivotExtension.ExtensionState.INTAKE1 : robot.pivotExtension.getState(PivotExtension.ExtensionState.class))
                .build();
    }

    public List<Task> justTopDropdown() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_TOP_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.TOP_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.INTAKE_EXTENSION)
//                .delay(400)
//                .moduleAction(robot.pivotExtension, robot.intakeSpecimen.getState(IntakeSpecimen.WristState.class).equals(IntakeSpecimen.WristState.SAMPLE_INTAKE) ?
//                        PivotExtension.ExtensionState.INTAKE1 : robot.pivotExtension.getState(PivotExtension.ExtensionState.class))
                .build();
    }
    public List<Task> shortDropdown() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.SHORT_EXTEND)
//                .delay(400)
//                .moduleAction(robot.pivotExtension, robot.intakeSpecimen.getState(IntakeSpecimen.WristState.class).equals(IntakeSpecimen.WristState.SAMPLE_INTAKE) ?
//                        PivotExtension.ExtensionState.INTAKE1 : robot.pivotExtension.getState(PivotExtension.ExtensionState.class))
                .build();
    }
    public List<Task> adjustDropdown() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_ADJUST_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.INTAKE_EXTENSION)
                .build();
    }

    public List<Task> clawDropdown() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE_NON_HOVER)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.CLAW_DROPDOWN)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.CLAW_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.INTAKE_EXTENSION)
                .delay(200)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.OPEN)
                .delay(1000)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.CLAW_INTAKE)
//                .delay(400)
//                .moduleAction(robot.pivotExtension, robot.intakeSpecimen.getState(IntakeSpecimen.WristState.class).equals(IntakeSpecimen.WristState.SAMPLE_INTAKE) ?
//                        PivotExtension.ExtensionState.INTAKE1 : robot.pivotExtension.getState(PivotExtension.ExtensionState.class))
                .build();
    }


    public List<Task> justExtake() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.EXTAKE)
                .build();
    }

    public List<Task> resetToIntakeHover() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE)
//                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.SLOW_EXTAKE)
//                .delay(50)
//                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .build();
    }

    public List<Task> autoIntakeReset() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .build();
    }

    public List<Task> autoRetryReset() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE_NON_HOVER)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .build();
    }

    public List<Task> autoIntakeResetAndExtend() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.SEMI_INTAKE)
                .build();
    }

    public List<Task> autoIntakeResetAndExtend(double ext) {
        return Builder.create()
                .executeCode(() -> robot.pivotExtension.getState(PivotExtension.ExtensionState.class).setValue(ext))
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.MANUAL)

                .build();
    }

    public List<Task> extendForPark() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.HANG_BEFORE)
                .build();
    }

    // only intakes (no dropdown)
    public List<Task> justIntake() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                .build();
    }


    public List<Task> resetFromHang() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.INIT)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INIT)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .moduleAction(robot.hang, Hang.LeftPTOState.RAISED)
                .moduleAction(robot.hang, Hang.RightPTOState.RAISED)
                .build();
    }

    public List<Task> resetWithoutIntakeReset(boolean isWrongColor) {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, isWrongColor ? IntakeSpecimen.PowerState.EXTAKE : robot.intakeSpecimen.getState(IntakeSpecimen.PowerState.class))
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BASKET_POST_DEPOSIT)
                .delay(100)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .await(() -> robot.pivotExtension.extensionPosition < 20)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE)

                .build();
    }

    public List<Task> limelightReset(boolean isWrongColor) {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, isWrongColor ? IntakeSpecimen.PowerState.EXTAKE : robot.intakeSpecimen.getState(IntakeSpecimen.PowerState.class))
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.LL_RESET)
                .delay(100)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
//                .addTaskList(sweep())
                .build();
    }

    public List<Task> resetFromLimelight() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE_NON_HOVER)
                //.delay(200)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.HOLDING)
                .build();
    }

    public List<Task> pushing() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INIT)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .build();
    }

    public List<Task> intake(boolean isAutoDrive) {
       return Builder.create()
                .addTaskList(resetToHover())
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.INTAKE_EXTENSION)
               .build();
    }

    public List<Task> basketIntake(boolean isAutoDrive) {
        return Builder.create()
                .addTaskList(intake(isAutoDrive))
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.BASKET_INTAKE_EXTENSION)
                .addTaskList(ActivateIntake())
                .build();
    }

    public List<Task> parkIntake() {
        return Builder.create()
                .addTaskList(intake(false))
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.INTAKE_EXTENSION)
                .addTaskList(ActivateIntake())
                .build();
    }

    public List<Task> specimenToIntake() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .await(() -> robot.pivotExtension.extensionPosition < 15)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE)
                .await(() -> robot.pivotExtension.pivotPosition < 30)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.INTAKE_EXTENSION)
                .build();
    }

    public List<Task> flip() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.FlipperState.UP)
                .delay(500)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.FlipperState.DOWN)
                .build();
    }

    public List<Task> scoreToPark() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .await(() -> robot.pivotExtension.extensionPosition < 15)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.PARK)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.PARK)
                .build();
    }

    public List<Task> resetBasketAutoToIntake() {
        return Builder.create()
                .await(() -> robot.pivotExtension.pivotPosition < PivotExtension.PivotState.BASKET_POST_DEPOSIT.getValue() + 3)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .await(() -> robot.pivotExtension.extensionPosition < 30)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)
//                .await(() -> robot.pivotExtension.pivotPosition < 17)
//                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.AUTO_EXTEND)
                .build();
    }

    public List<Task> resetToHover() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BASKET_POST_DEPOSIT)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.OPEN)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .delay(200)
                .await(() -> robot.pivotExtension.extensionPosition < 17)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE)
                .await(() -> robot.pivotExtension.pivotPosition < 26)
                .build();
    }

    public List<Task> resetBasketAutoToHover() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BASKET_POST_DEPOSIT)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.OPEN)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .delay(200)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .await(() -> robot.pivotExtension.extensionPosition < 17)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.SLOW_EXTAKE)
                .delay(100)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .build();
    }

    public List<Task> scoreToAda() {
        return Builder.create()
//                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.SCORE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .await(() -> robot.pivotExtension.extensionPosition < 5)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.ADA_LOW)
                .build();
    }

    public List<Task> adaScoreMacro() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.SCORE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .delay(depositWaitTime)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .await(() -> robot.pivotExtension.extensionPosition < 15)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HANG_AFTER)
                .build();
    }

    public List<Task> runIntake() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                .build();
    }


    public List<Task> highBasketExtensionActive(boolean doubleExtake) {

        double extend = 45;
        ElapsedTime timer = new ElapsedTime();

        Builder b = Builder.create();
        if (doubleExtake) {
           b
                   .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.EXTAKE)
                   .executeCode(timer::reset)
                   .await(() -> timer.milliseconds() > 30);
        }

        return b

                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.HOLDING)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_READY)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HIGH_BASKET_ACTIVE)
                .await(() -> (robot.pivotExtension.pivotPosition > extend))
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE)
                .await(() -> (robot.pivotExtension.extensionPosition > PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE.getValue() - 20))
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_ACTIVE_DEPOSIT)
                .build();
    }

    public List<Task> scoreWhenReady(double e4) {
        return Builder.create()
                .await(() -> robot.pivotExtension.extensionPosition > PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE.getValue() - e4)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.SCORE_AUTO)
                .delay(300)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BASKET_POST_DEPOSIT)
                .build();
    }

    public List<Task> highBasketExtensionActive(boolean isExtendEarly, double extension) {

        double extend = isExtendEarly ? 35 : 35;

        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_READY)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.HOLDING)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HIGH_BASKET_ACTIVE)
                .await(() -> (robot.pivotExtension.pivotPosition > extend))
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE)
                .await(() -> (robot.pivotExtension.extensionPosition > PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE.getValue() - 20))
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_ACTIVE_DEPOSIT)
                .build();
    }

    public List<Task> highBasketExtensionClaw(boolean isExtendEarly) {

        double extend = isExtendEarly ? 50 : 40;

        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_READY)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.HOLDING)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HIGH_BASKET_ACTIVE)
                .await(() -> (robot.pivotExtension.pivotPosition > extend))
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE)
                .await(() -> (robot.pivotExtension.extensionPosition > PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE.getValue() - 20))
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_CLAW_DEPOSIT)
                .build();
    }

    public List<Task> highBasketExtensionAngled(boolean isExtendEarly) {

        double extend = isExtendEarly ? 50 : 40;

        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_READY)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.HOLDING)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HIGH_BASKET_ACTIVE)
                .await(() -> (robot.pivotExtension.pivotPosition > extend))
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.HIGH_BASKET_HIGHER)
                .await(() -> (robot.pivotExtension.extensionPosition > PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE.getValue() - 20))
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_ANGLED_DEPOSIT)
                .build();
    }

    public List<Task> highBasketExtensionCycle(boolean doubleExtake) {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.BAR_PASS_INTAKE_NON_HOVER)
                .delay(200)
                .addTaskList(highBasketExtensionActive(doubleExtake))
                .build();

    }

    public List<Task> lowBasketExtensionIntake() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.HOLDING)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HIGH_BASKET_ACTIVE)
                .await(() -> (robot.pivotExtension.pivotPosition > 60))
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.LOW_BASKET_INTAKE)
                .await(() -> (robot.pivotExtension.pivotPosition > robot.pivotExtension.pivotTarget - 25 && robot.pivotExtension.extensionPosition > robot.pivotExtension.extensionTarget - 2))
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_ACTIVE_LOW_DEPOSIT)
                .build();

    }

    public List<Task> lowBasketExtensionClaw() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.PRE_LOW_BASKET_FROM_INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.HOLDING)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HIGH_BASKET_CLAW)
                .await(() -> (robot.pivotExtension.pivotPosition > 35))
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.LOW_BASKET_CLAW)
                .await(() -> (robot.pivotExtension.pivotPosition > robot.pivotExtension.pivotTarget - 25 && robot.pivotExtension.extensionPosition > robot.pivotExtension.extensionTarget - 2))
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_CLAW_DEPOSIT)
                .build();
    }

    public List<Task> specimenPreScore() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_OLD_DEPOSIT)
                .build();
    }

    public List<Task> specimenScore(boolean goToBasketIntakeAfter) {
        Builder b = Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.SPECIMEN_OVER)
                .await(() -> robot.pivotExtension.extensionPosition > PivotExtension.ExtensionState.SPECIMEN_UNDER.getValue() + 1)
                .delay(300)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.OPEN)
                .delay(100)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.SPECIMEN_PICKUP)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_PICKUP)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.SPECIMEN_PICKUP);

        if (goToBasketIntakeAfter)
            b
                    .delay(300)
                    .addTaskList(basketIntake(false));

        return b.build();
    }


    public List<Task> specimenFrontSetup() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.FRONT_SPECIMEN_BEFORE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.FRONT_SPECIMEN)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_FRONT_DEPOSIT)
                .build();
    }

    public List<Task> specimenFrontScore() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.OPEN)
//                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.FRONT_SPECIMEN_AFTER)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .build();
    }

    public List<Task> specimenScorePreload() {
        AtomicReference<Double> y = new AtomicReference<>(0.0);
        return Builder.create()
                .executeCode(() -> y.set(robot.pose.position.y))
                .await(() -> robot.pose.position.y < (y.get() - 1.2))
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.OPEN)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
                .build();
    }

    public List<Task> specimenGoToPickupYeet() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.SPECIMEN_PICKUP_PRELOAD)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.OPEN)


                .delay(500)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_PICKUP)
                .delay(450)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.SLOW_EXTAKE)
                .delay(700)

                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)

                .build();
    }

    public List<Task> specimenGoToPickupInit() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.SPECIMEN_PICKUP)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_PICKUP)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.OPEN)
                .build();
    }

    public List<Task> specimenGoToPickup() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.SPECIMEN_PICKUP)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.SPECIMEN_PICKUP)
                .await(() -> robot.pivotExtension.extensionPosition > PivotExtension.ExtensionState.SPECIMEN_PICKUP.getValue() - 1)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_PICKUP)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.OPEN)
                .build();
    }

    public List<Task> specimenPickup() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.YEET_THIRD)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .delay(200)
//                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.SPECIMEN_RAISE)
//                .delay(400)
//                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.LOOSE_CLAW)
//                .delay(600)
//                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.SPECIMEN_DEPOSIT)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .delay(200)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.SPECIMEN_UNDER)
                .delay(150)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_DEPOSIT)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.LOOSE_CLAW)
//                .delay(600)
//                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .build();
    }


    public List<Task> specimenPickupTele() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .delay(400)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.SPECIMEN_DEPOSIT)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .delay(200)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.LOOSE_CLAW)
                .delay(200)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_DEPOSIT)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.SPECIMEN_UNDER)
                .delay(600)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .build();
    }


    public List<Task> specimenSetupPreload() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_DEPOSIT)

                // .await(() -> robot.pivotExtension.extensionPosition < Context.extensionWaitToPivotIntake)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.SPECIMEN_DEPOSIT)
                .await(() -> robot.pivotExtension.pivotPosition > 50)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.SPECIMEN_UNDER)
                // .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.OPEN)
                .build();
    }


    public List<Task> Ada() {
        return Builder.create()
                .executeCode(() -> PivotExtension.MOTION_PROFILE.max_decel = 200)
                .executeCode(() -> PivotExtension.MOTION_PROFILE.max_accel = 200)
                .executeCode(() -> PivotExtension.MOTION_PROFILE.p /= 2)

                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.ADA)
                .delay(700)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INIT)
                .delay(700)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.ADA)
                .delay(700)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INIT)
                .delay(1000)
                .executeCode(() -> PivotExtension.MOTION_PROFILE.max_decel = 600)
                .executeCode(() -> PivotExtension.MOTION_PROFILE.max_accel = 500)

                .executeCode(() -> PivotExtension.MOTION_PROFILE.p *= 2)


                .executeCode(() -> robot.setDrivetrainPowers(0.5, 0, 0, 0.5))
                .delay(700)
                .executeCode(() -> robot.setDrivetrainPowers(-0.5, 0, 0, -0.5))
                .delay(700)
                .executeCode(() -> robot.setDrivetrainPowers(0.5, 0, 0, 0.5))
                .delay(700)
                .executeCode(() -> robot.setDrivetrainPowers(-0.5, 0, 0, -0.5))
                .delay(700)
                .executeCode(() -> robot.setDrivetrainPowers(0, 0, 0, 0))
                .delay(200)

                .executeCode(() -> Hang.LeftPitchState.INIT.setValue(0))
                .executeCode(() -> Hang.RightPitchState.INIT.setValue(0))
                .moduleAction(robot.intakeSpecimen, Hang.LeftPitchState.INIT)
                .moduleAction(robot.intakeSpecimen, Hang.RightPitchState.INIT)
                .delay(200)
                .executeCode(() -> Hang.LeftPitchState.INIT.setValue(1))
                .executeCode(() -> Hang.RightPitchState.INIT.setValue(1))
                .delay(200)
                .executeCode(() -> Hang.LeftPitchState.INIT.setValue(0))
                .executeCode(() -> Hang.RightPitchState.INIT.setValue(0))
                .delay(200)
                .executeCode(() -> Hang.LeftPitchState.INIT.setValue(1))
                .executeCode(() -> Hang.RightPitchState.INIT.setValue(1))
                .delay(200)
                .executeCode(() -> Hang.LeftPitchState.INIT.setValue(0))
                .executeCode(() -> Hang.RightPitchState.INIT.setValue(0))

                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.ADA)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_PICKUP)
                .delay(1000)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.INIT)
                .delay(750)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_PICKUP)
                .delay(500)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.INIT)
                .delay(400)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.ADA_LOW)

                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_OLD_DEPOSIT)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.OPEN)
                .delay(500)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)
                .delay(500)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.OPEN)
                .delay(500)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.CLOSED)

                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.ADA)
                .await(() -> (robot.pivotExtension.pivotPosition > 50))
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE)
                .await(() -> (robot.pivotExtension.extensionPosition > 28))
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_ACTIVE_DEPOSIT)
                .delay(1600)

                .addTaskList(scoreToAda())

                .build();
    }

    public List<Task> hangSetup() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.OFF)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.HANG_PREP)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HANG_BEFORE)
                .delay(1000)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.HANG_BEFORE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.HANG_SETUP)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.ClawState.LOOSE_CLAW)
                .build();
    }

    public List<Task> hangLow() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.HANG_HANGING_1)
                //turn wrist up so doesnt hit extension
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.HANG_AFTER)
                .delay(200)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HANG_PUSHUP)

                .delay(500)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HANG_AFTER)
                .moduleAction(robot.hang, Hang.LeftPTOState.DOWN)
                .moduleAction(robot.hang, Hang.RightPTOState.DOWN)
                .build();
    }

    public List<Task> hangLevel2() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .delay(400)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HANG_LEVEL_2)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.HANG_LEVEL_2)

                .build();
    }

//    public List<Task> resetHang() {
//        return Builder.create()
//                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HANG_LEVEL_3)
//                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.RESET)
////                .moduleAction(robot.hang, Hang.RotateState.OFF)
//                .build();
//    }

    public List<Task> basketScoreToSpecimenPickup() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.SCORE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .delay(depositWaitTime)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.SPECIMEN_PICKUP)
                .await(() -> robot.pivotExtension.extensionPosition < 30)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.SPECIMEN_PICKUP)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SPECIMEN_PICKUP)
                .build();
    }

    public List<Task> level1Hang() {
        return Builder.create()
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HANG_BEFORE)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.HANG_AFTER)

                .build();
    }

    public List<Task> score() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.SCORE)
                .build();
    }

    public List<Task> scoreCycles() {
        return Builder.create()
                .await(() -> Math.abs(robot.pivotExtension.extensionPosition - robot.pivotExtension.extensionTarget) < 1)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.SCORE_AUTO)
                .build();
    }

    public List<Task> sweep() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.RightSweeperState.OUT)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.LeftSweeperState.OUT)
                .delay(400)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.RightSweeperState.IN)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.LeftSweeperState.IN)
                .build();
    }

    public List<Task> light() {
        return Builder.create()
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.LightState.ON)
                .delay(1000)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.LightState.OFF)
                .build();
    }

    public List<Task> sweepAndExtend(boolean weep, int retries) {
        Builder b =  Builder.create();
        if (weep) {
                b.moduleAction(robot.intakeSpecimen, IntakeSpecimen.RightSweeperState.OUT)
                    .moduleAction(robot.intakeSpecimen, IntakeSpecimen.LeftSweeperState.OUT)
                    .delay(500)
            ;
        }
        return  b
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.AUTO_EXTEND)
                .moduleAction(robot.pivotExtension, PivotExtension.PivotState.INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                .await(() -> Math.abs(Math.toDegrees(robot.pose.heading.toDouble())) < 15)
                .executeCode(() -> Limelight.isDoneSweeping = true)
                .delay(300)
                .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.MANUAL)
                .executeCode(() -> PivotExtension.ExtensionState.MANUAL.setValue(Math.min(PivotExtension.ExtensionState.START_EXTENSION_AUTO.getValue() + retries * 3, PivotExtension.ExtensionState.INTAKE_EXTENSION.getValue())))
                .build();


                /*
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.RightSweeperState.IN)
                .moduleAction(robot.intakeSpecimen, IntakeSpecimen.LeftSweeperState.IN)
                .delay(600)
                 */


    }
}
