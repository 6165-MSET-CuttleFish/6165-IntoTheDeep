package org.firstinspires.ftc.teamcode.pathing;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.control.AccelVelConstraint;
import org.firstinspires.ftc.teamcode.architecture.markers.Axis;
import org.firstinspires.ftc.teamcode.architecture.markers.Marker;
import org.firstinspires.ftc.teamcode.architecture.markers.SpatialMarker;
import org.firstinspires.ftc.teamcode.architecture.markers.TemporalMarker;
import org.firstinspires.ftc.teamcode.architecture.modules.IntakeSpecimen;
import org.firstinspires.ftc.teamcode.architecture.modules.Limelight;
import org.firstinspires.ftc.teamcode.architecture.modules.PivotExtension;
import org.firstinspires.ftc.teamcode.pathing.path_architecture.Path;

/**
 * see SpecimenPath for comments
 */
public class CyclePath extends Path {
    public static double accel = 50, decel = 50, vel = 60, vel2 = 60, decel2 = 60, accel2 = 70;
    public static Pose2d basketPose = new Pose2d(-53, -59, Math.toRadians(45));
    public static Pose2d cyclePose = new Pose2d(-18.5, -4, Math.toRadians(0));
    ElapsedTime intakeTimer = new ElapsedTime();

    public CyclePath(Pose2d startPose) {
        super(startPose);
    }

    public Marker extend(double time, boolean isEarly) {
        return new TemporalMarker(time, robot.actions.highBasketExtensionActive(isEarly));
    }

    public Marker extendCycle(double time, boolean doubleExtake) {
        return new TemporalMarker(time, robot.actions.highBasketExtensionCycle(doubleExtake));
    }

    public Marker scoreOnCycles(double dist) {
        return new SpatialMarker(cyclePose.position.y + dist, Axis.Y, robot.actions.scoreCycles());
    }


    public Marker resetPivotIntake(double timeMs) {
        return new TemporalMarker(timeMs/1000.0, robot.actions.resetBasketAutoToIntake());
    }

//    public Marker activateIntake(double time) {
//        return new TemporalMarker(time, robot.actions.activateIntake());
//    }

    public Marker[] submersibleDropdown(double extendPos, double dropdownPos) {
        return new Marker[]{
                new SpatialMarker(extendPos, Axis.Y, () -> robot.pivotExtension.setState(PivotExtension.ExtensionState.YEET)),
//                new SpatialMarker(dropdownPos, Axis.Y, robot.actions.activateIntake())
        };
    }

    @Override
    public Action createPath() {

        actionBuilder
                .addAction(() -> robot.intakeSpecimen.setState(IntakeSpecimen.ClawState.OPEN))
                .addAction(robot.actions.highBasketExtensionActive(true))
                .addDelayMs(250)
                .addPath(preload())

                .addAwait(() -> robot.pivotExtension.extensionPosition > PivotExtension.ExtensionState.HIGH_BASKET_ACTIVE.getValue() - 1)
                .addAction(robot.actions.score())
                .addDelayMs(200);

        for (int i = 0; i < 100; i++) {
            actionBuilder
            .addDelayMs(550)
                    .addAction(robot.actions.resetWithoutIntakeReset(true))
                    .addPath(intakeCycle(),
                            new TemporalMarker(0, robot.actions.resetBasketAutoToHover())
//                        new SpatialMarker(-35, Axis.X, () -> robot.intakeSpecimen.setState(IntakeSpecimen.LeftSweeperState.OUT)),
//                        new SpatialMarker(-35, Axis.X, () -> robot.intakeSpecimen.setState(IntakeSpecimen.RightSweeperState.OUT)),
//                        new SpatialMarker(-21, Axis.X, robot.actions.extendAndIntake(3, PivotExtension.ExtensionState.INTAKE_EXTENSION.getValue()))
                    )
                    .addDelayMs(200)
//                .addAction(() -> robot.intakeSpecimen.setState(IntakeSpecimen.LeftSweeperState.IN))
//                .addAction(() -> robot.intakeSpecimen.setState(IntakeSpecimen.RightSweeperState.IN))

                    .addAction(new Limelight.SearchForSample(false, i==0, 0))

//                .addAction(() -> robot.intakeSpecimen.setState(IntakeSpecimen.LeftSweeperState.IN))
                    .addAction(new basketWrapper())
                    .addAction(() -> robot.intakeSpecimen.setState(IntakeSpecimen.LeftSweeperState.IN))
                    .addDelayMs(250)
                    .addAction(robot.actions.resetWithoutIntakeReset(true));
        }
        return  actionBuilder.build();
    }

    public TrajectoryActionBuilder preload() {
        return builder
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(basketPose, basketPose.heading.toDouble() + Math.toRadians(180));
    }

    public TrajectoryActionBuilder intake1() {
        return preload().endTrajectory().fresh()
//                 .afterTime(0.5, () -> farEnoughInTrajectory = true)
                .strafeToSplineHeading(new Vector2d(-55, -46), Math.toRadians(55), AccelVelConstraint.VelConstraint(vel), AccelVelConstraint.AccelConstraint(-decel, accel));
        //.splineToConstantHeading(new Vector2d(-48, -37), Math.toRadians(73), AccelVelConstraint.VelConstraint(vel), AccelVelConstraint.AccelConstraint(-decel, accel));
    }

    public TrajectoryActionBuilder basket1() {
        return intake1().endTrajectory().fresh()
                .setTangent(Math.toRadians(-105))
                .splineToLinearHeading(basketPose, basketPose.heading.toDouble() + Math.toRadians(180), AccelVelConstraint.VelConstraint(vel2), AccelVelConstraint.AccelConstraint(-decel2, accel2));
    }

    public TrajectoryActionBuilder intake2() {
        return basket1().endTrajectory().fresh()
//                .afterTime(0.5, () -> farEnoughInTrajectory = true)
                .strafeToSplineHeading(new Vector2d(-38, -35), Math.toRadians(150), AccelVelConstraint.VelConstraint(vel), AccelVelConstraint.AccelConstraint(-decel, accel));
        // .splineToConstantHeading(new Vector2d(-55, -34.5), Math.toRadians(90), AccelVelConstraint.VelConstraint(vel), AccelVelConstraint.AccelConstraint(-decel, accel));
    }

    public TrajectoryActionBuilder basket2() {
        return intake2().endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(basketPose, basketPose.heading.toDouble() + Math.toRadians(180), AccelVelConstraint.VelConstraint(vel), AccelVelConstraint.AccelConstraint(-decel, accel));
    }

    public TrajectoryActionBuilder intake3() {

        return basket2().endTrajectory().fresh()
//                .afterTime(0.5, () -> farEnoughInTrajectory = true)
                .strafeToSplineHeading(new Vector2d(-45, -36), Math.toRadians(150), AccelVelConstraint.VelConstraint(vel), AccelVelConstraint.AccelConstraint(-decel, accel));
        // .splineToConstantHeading(new Vector2d(-55, -41), Math.toRadians(130), AccelVelConstraint.VelConstraint(vel), AccelVelConstraint.AccelConstraint(-decel, accel));
    }

    public TrajectoryActionBuilder basket3() {
        return intake3().endTrajectory().fresh()
                .setTangent(Math.toRadians(-120))
                .splineToLinearHeading(basketPose, basketPose.heading.toDouble() + Math.toRadians(180), AccelVelConstraint.VelConstraint(vel), AccelVelConstraint.AccelConstraint(-decel, accel));
    }

    public TrajectoryActionBuilder intakeCycle() {
        return basket3().endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(cyclePose, Math.toRadians(0), AccelVelConstraint.VelConstraint(100), AccelVelConstraint.AccelConstraint(-40, 70));
    }

    public TrajectoryActionBuilder basketCycle() {
        return robot.actionBuilder(cyclePose)
                .setTangent(Math.toRadians(210))
                .splineToLinearHeading(new Pose2d(basketPose.position.x + 1, basketPose.position.y - 0.5, basketPose.heading.toDouble()), Math.toRadians(250), AccelVelConstraint.VelConstraint(70), AccelVelConstraint.AccelConstraint(-35, 60));
    }

    public TrajectoryActionBuilder park() {
        return basketCycle().endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(cyclePose.position.x + 3, cyclePose.position.y - 5, cyclePose.heading.toDouble()), Math.toRadians(0), AccelVelConstraint.VelConstraint(80), AccelVelConstraint.AccelConstraint(-60, 80));
                .splineToLinearHeading(new Pose2d(basketPose.position.x + 7, basketPose.position.y + 7, basketPose.heading.toDouble()), Math.toRadians(0), AccelVelConstraint.VelConstraint(80), AccelVelConstraint.AccelConstraint(-60, 80));
    }

    public static class basketWrapper implements Action {
        boolean firstRun, runMarker;

        Action action;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!firstRun) {
                firstRun = true;
                action = robot.actionBuilder(robot.pose)
                        .setTangent(Math.toRadians(210))
                        .splineToLinearHeading(new Pose2d(basketPose.position.x + 1, basketPose.position.y - 0.5, basketPose.heading.toDouble()), Math.toRadians(250), AccelVelConstraint.VelConstraint(70), AccelVelConstraint.AccelConstraint(-35, 60))
                        .build();

            }

            boolean running = action.run(telemetryPacket);

            if ((robot.pose.position.y < basketPose.position.y + 6 && !runMarker) || !running) {
                runMarker = true;
                robot.runAction(robot.actions.scoreCycles());
            }



            return running;

        }
    }
    @Override
    public Action createPathMeep() {
        return actionBuilder
                .addPath(preload())
                .addPath(intake1())
                .addPath(basket1())
                .addPath(intake2())
                .addPath(basket2())
                .addPath(intake3())
                .addPath(basket3())
                .addPath(intakeCycle())
                .addPath(basketCycle())
                .addPath(park())

                .build();
    }


}
