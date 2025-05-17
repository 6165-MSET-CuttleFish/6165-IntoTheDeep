package org.firstinspires.ftc.teamcode.pathing;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.architecture.control.AccelVelConstraint;
import org.firstinspires.ftc.teamcode.architecture.markers.InstantMarker;
import org.firstinspires.ftc.teamcode.architecture.modules.IntakeSpecimen;
import org.firstinspires.ftc.teamcode.architecture.modules.Limelight;
import org.firstinspires.ftc.teamcode.architecture.modules.PivotExtension;
import org.firstinspires.ftc.teamcode.pathing.path_architecture.ActionBuilder;
import org.firstinspires.ftc.teamcode.pathing.path_architecture.Path;
import org.firstinspires.ftc.teamcode.roadrunner.Drive;

/**
 * see SpecimenPath for comments
 */
public class BasketPath extends Path {
    public static double intakeAccel = 40, intakeDecel = 10, intakeVel = 10, basketVel = 50, basketDecel = 50, basketAccel = 50;
    public static double awaitExtensionDeposit = 1, stallRetryCountThresh = 2;
    public static Pose2d basketPose = new Pose2d(-52, -61, Math.toRadians(45));
    public static Vector2d basketCyclePose = new Vector2d(-56.2, -54.4);
    public static double basketCycleEndTangent = 70;
    public static Pose2d cyclePose = new Pose2d(-21, -3, Math.toRadians(0));
    public static Vector2d intake1 = new Vector2d(-42, -36);
    public static Vector2d intake2 = new Vector2d(-55, -36);
    public static Vector2d intake3 = new Vector2d(-59, -35);

    public BasketPath(Pose2d startPose) {
        super(startPose);
    }

    public ActionBuilder retryIntake(ActionBuilder actionBuilder) {
        return actionBuilder
                .addAwaitTimeout(() -> robot.intakeSpecimen.getCountStall() > 3, 500)
                .addAction(new ActionBuilder.conditionalAction(() -> robot.intakeSpecimen.getCountStall() < stallRetryCountThresh, robot.actions.autoRetryReset()))
//                .addDelayMs(1500)
                .addAction(new ActionBuilder.conditionalAction(() -> robot.intakeSpecimen.getCountStall() < stallRetryCountThresh, new Limelight.LimelightSearchForSample(true)));
    }

    @Override
    public Action createPath() {
        actionBuilder
                .addAction(() -> robot.intakeSpecimen.setState(IntakeSpecimen.ClawState.OPEN))
                .addAction(robot.actions.highBasketExtensionActive(false))
                .addAwait(() -> robot.pivotExtension.extensionPosition > 4)
                .addPath(preload(), new InstantMarker(robot.actions.scoreWhenReady(awaitExtensionDeposit)))

                .addAction(robot.actions.resetBasketAutoToIntake())
                .addAction(new Wrapper("intake1"));

        actionBuilder = retryIntake(actionBuilder)
                .addAction(robot.actions.highBasketExtensionActive(false))
                .addAwait(() -> robot.pivotExtension.extensionPosition > 6)
                .addAction(robot.actions.scoreWhenReady(awaitExtensionDeposit))
                .addAction(new Wrapper("basket1"))

                .addAction(robot.actions.resetBasketAutoToIntake())
                .addAction(new Wrapper("intake2"));

        actionBuilder = retryIntake(actionBuilder)
                .addAction(robot.actions.highBasketExtensionActive(false))
                .addAwait(() -> robot.pivotExtension.extensionPosition > 7)
                .addAction(robot.actions.scoreWhenReady(awaitExtensionDeposit))

                .addAction(new Wrapper("basket2"))
                .addAction(robot.actions.resetBasketAutoToIntake())
                .addAction(new Wrapper("intake3"));

        actionBuilder = retryIntake(actionBuilder)
                .addAction(robot.actions.highBasketExtensionActive(true))
                .addAwait(() -> robot.pivotExtension.extensionPosition > 6)
                .addAction(robot.actions.scoreWhenReady(awaitExtensionDeposit))

                .addAction(new Wrapper("basket3"))
//                .addAction(new Limelight.LimelightSearchForSample(true))
                .addAction(() -> Drive.PARAMS.axialGain = 25)
                .addAction(() -> Drive.PARAMS.axialVelGain = 2)
                .addAction(() -> Drive.PARAMS.lateralGain = 25)
                .addAction(() -> Drive.PARAMS.lateralVelGain = 2)
                .addAction(robot.actions.scoreWhenReady(awaitExtensionDeposit));

        for (int i = 1; i < 4; i++) {
            actionBuilder
                    .addAction(robot.actions.resetBasketAutoToHover())

                    .addAction(new Wrapper("intakeCycle" + i))
                    .addAction(new Limelight.SearchForSample(true, i==1, i))
//                    .addAction(new Limelight.SearchForSample(false, i==1, i))
                    .addAction(() -> robot.intakeSpecimen.setState(IntakeSpecimen.LeftSweeperState.IN))

                    .addAction(robot.actions.highBasketExtensionActive(true))
                    .addAction(new Wrapper("basketCycle"));
        }

        return actionBuilder
                .addAction(robot.actions.scoreWhenReady(awaitExtensionDeposit))

                .addDelayMs(10000).build();
    }

    public static TrajectoryActionBuilder preload() {
        return builder
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(basketPose, basketPose.heading.toDouble() + Math.toRadians(180));
    }

    public static TrajectoryActionBuilder intake1() {
        return robot.actionBuilder(robot.pose)
                .strafeToSplineHeading(intake1, Math.toRadians(90), AccelVelConstraint.VelConstraint(intakeVel-1), AccelVelConstraint.AccelConstraint(-intakeDecel-2, intakeAccel));

    }

    public static TrajectoryActionBuilder basket1() {
        return robot.actionBuilder(robot.pose)
                .setTangent(Math.toRadians(-105))
                .splineToLinearHeading(basketPose, basketPose.heading.toDouble() + Math.toRadians(180), AccelVelConstraint.VelConstraint(basketVel), AccelVelConstraint.AccelConstraint(-basketDecel, basketAccel));
    }

    public static TrajectoryActionBuilder intake2() {
        Pose2d startPose = robot.pose;
        double angle = Math.abs(Math.atan2(intake2.y - startPose.position.y, intake2.x - startPose.position.x));
        double distance = 5;
        return robot.actionBuilder(startPose)
                .strafeToSplineHeading(new Vector2d(-Math.sin(angle) * distance + intake2.x, -Math.cos(angle) * distance + intake2.y), Math.toRadians(90), AccelVelConstraint.VelConstraint(intakeVel), AccelVelConstraint.AccelConstraint(-intakeDecel, intakeAccel))
                .strafeTo(new Vector2d(intake2.x, intake2.y), AccelVelConstraint.VelConstraint(intakeVel), AccelVelConstraint.AccelConstraint(-intakeDecel, intakeAccel));
    }

    public static TrajectoryActionBuilder basket2() {
        return robot.actionBuilder(robot.pose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(basketPose, basketPose.heading.toDouble() + Math.toRadians(180), AccelVelConstraint.VelConstraint(basketVel), AccelVelConstraint.AccelConstraint(-basketDecel, basketAccel));
    }

    public static TrajectoryActionBuilder intake3() {
        return robot.actionBuilder(robot.pose)
                .strafeToSplineHeading(intake3, Math.toRadians(126), AccelVelConstraint.VelConstraint(intakeVel), AccelVelConstraint.AccelConstraint(-intakeDecel, intakeAccel));
    }

    public static TrajectoryActionBuilder basket3() {
        return robot.actionBuilder(robot.pose)
                .setTangent(Math.toRadians(-60))
                .splineToLinearHeading(new Pose2d(basketCyclePose.x, basketCyclePose.y, Math.toRadians(70)), Math.toRadians(70) + Math.toRadians(180), AccelVelConstraint.VelConstraint(30), AccelVelConstraint.AccelConstraint(-35, 35));
    }


    public static TrajectoryActionBuilder intakeCycle(int cycle, double heading) {
        double angleOffset = 0;


        if (cycle == 1) {
            angleOffset = 15;
        } else if (cycle == 2) {
            angleOffset = -15;
        }

        angleOffset = Math.toRadians(angleOffset);

        double x = (basketCyclePose.x) + (1/(Math.tan(Math.toRadians(heading))) * (cyclePose.position.y - 30 - basketCyclePose.y));

        return robot.actionBuilder(robot.pose)
                .strafeTo(new Vector2d(x, cyclePose.position.y - 30), AccelVelConstraint.VelConstraint(70), AccelVelConstraint.AccelConstraint(-60, 60))
                .splineToSplineHeading(new Pose2d(cyclePose.position, angleOffset), Math.toRadians(0), AccelVelConstraint.VelConstraint(65), AccelVelConstraint.AccelConstraint(-60, 60));    }

    public static TrajectoryActionBuilder basketCycle(double heading) {
        double x = basketCyclePose.x + (1/(Math.tan(Math.toRadians(heading))) * (cyclePose.position.y - 15 - basketCyclePose.y));

        Pose2d pose = robot.pose;

        return robot.actionBuilder(pose)
                .setTangent(Math.toRadians(190))
                .splineToSplineHeading(new Pose2d(x, cyclePose.position.y - 15, Math.toRadians(basketCycleEndTangent)), Math.toRadians(basketCycleEndTangent + 180), AccelVelConstraint.VelConstraint(30), AccelVelConstraint.AccelConstraint(-30, 30))
                .strafeTo(new Vector2d(basketCyclePose.x, basketCyclePose.y), AccelVelConstraint.VelConstraint(40), AccelVelConstraint.AccelConstraint(-40, 40));
    }

    public static class Wrapper implements Action {
        Action action;
        String path;

        double countStall = 0;
        boolean firstWrite = true;

        public Wrapper(String path) {
            this.path = path;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {



            if (firstWrite) {
                action = initializeAction(path);
                firstWrite = false;
            }
            boolean isPathRunning = action.run(telemetryPacket);

            if (path.contains("intake") && !path.contains("cycle")) {
                if (robot.intakeSpecimen.intake.isOverCurrent() &&  robot.pivotExtension.pivotPosition < 20) {
                    countStall++;
                } else if (countStall > 0) {
                    countStall--;
                }

                if ((countStall > 5 && robot.pivotExtension.pivotPosition < 20) || !isPathRunning) {
                    robot.setDrivetrainPowers(0, 0, 0, 0);
                    return false;
                }
            } else if (path.contains("basketCycle")) {
                if (Math.abs(robot.pose.position.y - basketCyclePose.y) < 2) {
                    robot.runAction(robot.actions.scoreWhenReady(awaitExtensionDeposit));
                }
            }

            return isPathRunning;
        }

        public Action initializeAction(String path) {
            TrajectoryActionBuilder b = robot.actionBuilder(new Pose2d(0,0,0));
            switch (path) {
                case "intake1":
                    b = intake1();
                    break;
                case "basket1":
                    b = basket1();
                    break;
                case "intake2":
                    b = intake2();
                    break;
                case "basket2":
                    b = basket2();
                    break;
                case "intake3":
                    b = intake3();
                    break;
                case "basket3":
                    b = basket3();
                    break;
                case "intakeCycle1":
                    b = intakeCycle(1, basketCycleEndTangent);
                    break;
                case "intakeCycle2":
                    b = intakeCycle(2, basketCycleEndTangent);
                    break;
                case "intakeCycle3":
                    b = intakeCycle(3, basketCycleEndTangent);
                    break;
                case "basketCycle":
                    b = basketCycle(basketCycleEndTangent);
                    break;
            }
            return b.build();
        }
    }

    @Override
    public Action createPathMeep() {
        return actionBuilder
                .addPath(preload())
                .build();
    }
}
