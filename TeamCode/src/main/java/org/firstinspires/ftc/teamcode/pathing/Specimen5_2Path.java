package org.firstinspires.ftc.teamcode.pathing;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.architecture.control.AccelVelConstraint;
import org.firstinspires.ftc.teamcode.architecture.markers.Axis;
import org.firstinspires.ftc.teamcode.architecture.markers.Marker;
import org.firstinspires.ftc.teamcode.architecture.markers.SpatialMarker;
import org.firstinspires.ftc.teamcode.architecture.markers.TemporalMarker;
import org.firstinspires.ftc.teamcode.pathing.path_architecture.Path;
@Config
public class Specimen5_2Path extends Path {
    public static double offset = -2;
    static Vector2d scoreVector = new Vector2d(8.5, -31);
    static Vector2d pickupVector = new Vector2d(36, -59);
    static Vector2d basketVector = new Vector2d(-51, -56);
    static Vector2d intake1 = new Vector2d(39.5, -36);
    static Vector2d place1 = new Vector2d(39, -41);
    static Vector2d intake2 = new Vector2d(51, -36.5);
    static Vector2d place2 = new Vector2d(49, -38);
    static Vector2d intake3 = new Vector2d(61, -35);
    static Vector2d basketIntake = new Vector2d(28, -64);


    public Specimen5_2Path(Pose2d startPose) {
        super(startPose);
    }

    public Marker specimenPickup(double pos) {
        return new SpatialMarker(pos, Axis.Y, robot.actions.specimenPickupTele());
    }

    /**
     * robot path w/ delays, markers, actions, paths
     */
    @Override
    public Action createPath() {
        actionBuilder
                .addAction(robot.actions.specimenFrontSetup())
                .addDelayMs(150)
                .addPath(preload(-3),
                        new SpatialMarker(-40, Axis.Y, robot.actions.specimenFrontScore())
                )
                .addPath(intake1(-3),
                        new TemporalMarker(0.2, robot.actions.firstExtendAndActivateIntake())
                )
                .addAction(robot.actions.autoIntakeResetAndExtend())

                .addPath(place1(),
                        new TemporalMarker(0.1, robot.actions.extendAndActivateExtake(300))
                )
                .addPath(intake2())
                .addAction(robot.actions.autoIntakeResetAndExtend())
                .addPath(place2(),
                        new TemporalMarker(0.1, robot.actions.extendAndActivateExtake(200))
                )
                .addPath(intake3())
                .addAction(robot.actions.autoIntakeReset())
                .addPath(pickup1(),
                        new TemporalMarker(0.05, robot.actions.specimenGoToPickupYeet()),
                        specimenPickup(pickupVector.y + 3)
                );

        for (int i = 1; i < 5; i++) {
            if (i == 1) {
                actionBuilder
                        .addPath(cycle1(offset * i),
                                new SpatialMarker(scoreVector.y - 5, Axis.Y, robot.actions.specimenScore(false))
                        );
            } else {
                actionBuilder
                        .addPath(cycle(offset * i),
                                new SpatialMarker(scoreVector.y - 5, Axis.Y, robot.actions.specimenScore(i==4))
                        );
            }
            if (i == 4) {
                actionBuilder
                        .addPath(basketIntake(offset * i));
            } else {
                actionBuilder
                        .addAction(new pickupWrapper());
            }
        }

        actionBuilder
                .addPath(basketCycle1(0),
                new TemporalMarker(0, robot.actions.highBasketExtensionActive(false)),
                new SpatialMarker(-50.5, Axis.X, robot.actions.score()));

        actionBuilder
                .addAction(new BasketIntake2Wrapper())
                .addAction(robot.actions.highBasketExtensionActive(false))
                .addAwait(() -> robot.pivotExtension.extensionPosition > 7)
                .addAction(robot.actions.scoreWhenReady(1))
                .addAction(new BasketCycle2Wrapper());


        return actionBuilder.addDelayMs(10000).build();
    }

    /**
     * use of TrajectoryActionBuilder
     * it takes the last TrajectoryActionBuilder (or if it's the first path, the path builder initialized in Path)
     * and it builds a path from that using all your known path methods (splineTo, lineToX, etc)
     * each one of these methods returns a TrajectoryActionBuilder as to "pass off" each path to the next
     * usually you would add a .build() which would bring all the paths together and return an Action
     * however, it's easier to take data from a TrajectoryActionBuilder than an Action, so we leave off the .build() until the very end
     * you can't grab the endPose of a generic Action, we need to take the end state and pass it in as start state for next path
     * that is done as "Last_TrajectoryActionBuilder().endTrajectory().fresh"
     */
    public static TrajectoryActionBuilder preload(double offset) {
        return builder
                .strafeToConstantHeading(
                        new Vector2d(scoreVector.x + offset + 2, -36),
                        AccelVelConstraint.VelConstraint(70),
                        AccelVelConstraint.AccelConstraint(-90, 70)
                );
    }

    public static TrajectoryActionBuilder intake1(double offset) {
        double angle = Math.toRadians(21);
        double xOffsetFromPickupSpline = -8;
        double yOffsetFromPickupSpline = xOffsetFromPickupSpline * tan(angle);

        return preload(offset).endTrajectory().fresh()
                .setTangent(toRadians(-70))
                .splineToSplineHeading(
                        new Pose2d(intake1.x + xOffsetFromPickupSpline, intake1.y + yOffsetFromPickupSpline, angle),
                        angle,
                        AccelVelConstraint.VelConstraint(60),
                        AccelVelConstraint.AccelConstraint(-50, 70)
                )
                .splineToConstantHeading(
                        intake1,
                        angle,
                        AccelVelConstraint.VelConstraint(60),
                        AccelVelConstraint.AccelConstraint(-50, 50)
                );
    }

    public static TrajectoryActionBuilder place1() {
        return intake1(5).endTrajectory().fresh()
                .strafeToLinearHeading(place1, Math.toRadians(-60));
    }

    public static TrajectoryActionBuilder intake2() {
        double dist = 5;
        double angle = Math.atan2(intake2.y - place1.y, intake2.x - place1.x);

        return place1().endTrajectory().fresh()
                .setTangent(angle)
                .strafeToSplineHeading(
                        new Vector2d(intake2.x - dist, intake2.y - dist * Math.tan(angle)),
                        angle,
                        AccelVelConstraint.VelConstraint(60),
                        AccelVelConstraint.AccelConstraint(-38, 40)
                )
                .splineToConstantHeading(
                        intake2,
                        angle,
                        AccelVelConstraint.VelConstraint(60),
                        AccelVelConstraint.AccelConstraint(-40, 40)
                );
    }

    public static TrajectoryActionBuilder place2() {
        return intake2().endTrajectory().fresh()
                .strafeToLinearHeading(place2, Math.toRadians(-60));
    }

    public static TrajectoryActionBuilder intake3() {
        double dist = 5;
        double angle = Math.atan2(intake3.y - place2.y, intake3.x - place2.x);

        return place2().endTrajectory().fresh()
                .setTangent(angle)
                .strafeToSplineHeading(
                        new Vector2d(intake3.x - dist, intake3.y - dist * Math.tan(angle)),
                        angle,
                        AccelVelConstraint.VelConstraint(60),
                        AccelVelConstraint.AccelConstraint(-40, 40)
                )
                .splineToConstantHeading(
                        intake3,
                        angle,
                        AccelVelConstraint.VelConstraint(60),
                        AccelVelConstraint.AccelConstraint(-40, 40)
                );
    }

    public static TrajectoryActionBuilder pickup1() {
        return intake3().endTrajectory().fresh()
                .setTangent(toRadians(180))
                .splineToSplineHeading(
                        new Pose2d(pickupVector.x + 9.5, pickupVector.y, toRadians(-90)),
                        toRadians(-90),
                        AccelVelConstraint.VelConstraint(30),
                        AccelVelConstraint.AccelConstraint(-60, 60)
                )
                .splineToConstantHeading(
                        new Vector2d(pickupVector.x + 9.5, pickupVector.y+0.4),
                        toRadians(-90),
                        AccelVelConstraint.VelConstraint(26),
                        AccelVelConstraint.AccelConstraint(-32, 60)
                );
    }

    public static TrajectoryActionBuilder cycle1(double offset) {
        return pickup1().endTrajectory().fresh()
                .strafeToConstantHeading(
                        new Vector2d(scoreVector.x + offset + 3, scoreVector.y - 9),
                        AccelVelConstraint.VelConstraint(50),
                        AccelVelConstraint.AccelConstraint(-50, 70)
                )
                .splineToConstantHeading(
                        new Vector2d(scoreVector.x + offset, scoreVector.y),
                        Math.toRadians(90),
                        AccelVelConstraint.VelConstraint(50),
                        AccelVelConstraint.AccelConstraint(-50, 50)
                );
    }

    public TrajectoryActionBuilder cycle(double offset) {
        return pickup(0).endTrajectory().fresh()
                .strafeToConstantHeading(
                        new Vector2d(scoreVector.x + offset + 1.5, scoreVector.y - 9),
                        AccelVelConstraint.VelConstraint(70),
                        AccelVelConstraint.AccelConstraint(-60, 70)
                )
                .splineToConstantHeading(
                        new Vector2d(scoreVector.x + offset, scoreVector.y),
                        Math.toRadians(90),
                        AccelVelConstraint.VelConstraint(65),
                        AccelVelConstraint.AccelConstraint(-55, 60)
                );
    }

    // UNUSED RIGHT NOW
    public TrajectoryActionBuilder pickup(double offset) {
        return cycle1(offset).endTrajectory().fresh()
                .setTangent(Math.toRadians(-60))
                .splineToSplineHeading(
                        new Pose2d(pickupVector, Math.toRadians(-90)),
                        Math.toRadians(-80),
                        AccelVelConstraint.VelConstraint(50),
                        AccelVelConstraint.AccelConstraint(-22, 60)
                );
    }

    public static TrajectoryActionBuilder basketIntake(double offset) {
        return cycle1(offset).endTrajectory().fresh()
                .strafeToSplineHeading(
                        basketIntake,
                        Math.toRadians(-9),
                        AccelVelConstraint.VelConstraint(70),
                        AccelVelConstraint.AccelConstraint(-58, 70)
                );
    }

    public static TrajectoryActionBuilder basketCycle1(double offset) {
        double x = -20;
        double y = Math.abs(Math.tan(Math.toRadians(9)) * (basketIntake.x - x)) + basketIntake.y;
        return basketIntake(offset).endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(x, y))
                .splineToSplineHeading(new Pose2d(basketVector.x + 3, basketVector.y - 2, Math.toRadians(20)),
                        Math.toRadians(190),
                        AccelVelConstraint.VelConstraint(60),
                        AccelVelConstraint.AccelConstraint(-60, 55)
                );
    }

    public static TrajectoryActionBuilder basketIntake2(Pose2d startPose) {
        return robot.actionBuilder(startPose)
                .strafeTo(new Vector2d(-50, -48), AccelVelConstraint.VelConstraint(60), AccelVelConstraint.AccelConstraint(-60, 60))
                .splineToSplineHeading(new Pose2d(-43, -35.5, Math.toRadians(90)), Math.toRadians(90), AccelVelConstraint.VelConstraint(10), AccelVelConstraint.AccelConstraint(-10, 10));
    }

    public static TrajectoryActionBuilder basketCycle2(Pose2d startPose) {
        return robot.actionBuilder(startPose)
                .setTangent(Math.toRadians(-105))
                .splineToLinearHeading(new Pose2d(-53, -58, Math.toRadians(45)), Math.toRadians(45) + Math.toRadians(180), AccelVelConstraint.VelConstraint(50), AccelVelConstraint.AccelConstraint(-50, 50));
    }

    public static class BasketIntake2Wrapper implements Action {
        ElapsedTime timer;

        Action action;

        double countStall = 0;

        boolean firstWrite = true;

        boolean firstAction = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstWrite) {
                action = basketIntake2(robot.pose).build();
                timer = new ElapsedTime();
                timer.reset();
                firstWrite = false;
            }

            if (timer.milliseconds() > 300 && !firstAction) {
                robot.runAction(robot.actions.resetBasketAutoToIntake());
                firstAction = true;
            }

            if (robot.intakeSpecimen.intake.isOverCurrent() && robot.pivotExtension.pivotPosition < 20) {
                countStall++;
            } else if (countStall > 0) {
                countStall--;
            }

            if ((countStall > 5 && robot.pivotExtension.pivotPosition < 20) || !action.run(telemetryPacket)) {
                robot.setDrivetrainPowers(0, 0, 0, 0);
                return false;
            }

            return true;
        }
    }

    public static class BasketCycle2Wrapper implements Action {
        Action action;

        boolean firstWrite = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstWrite) {
                action = basketCycle2(robot.pose).build();
                firstWrite = false;
            }

            return action.run(telemetryPacket);
        }
    }

    public static class pickupWrapper implements Action {
        boolean firstRun, runMarker;
        Action action;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!firstRun) {
                firstRun = true;
                double time = System.nanoTime();
                action = robot.actionBuilder(robot.pose)
                        .setTangent(Math.toRadians(-60))
                        .splineToSplineHeading(
                                new Pose2d(pickupVector, Math.toRadians(-90)),
                                Math.toRadians(-70),
                                AccelVelConstraint.VelConstraint(49),
                                AccelVelConstraint.AccelConstraint(-40, 48)
                        )
                        .build();

                RobotLog.e("TIMEEEEEE: " + ((System.nanoTime() - time) / Math.pow(10, 9)));
            }

            boolean running = action.run(telemetryPacket);

            if ((robot.pose.position.y < pickupVector.y + 3 && !runMarker) || !running) {
                runMarker = true;
                robot.runAction(robot.actions.specimenPickupTele());
            }

            return running;
        }
    }

    /**
     * meepmeep path w/ delays and paths
     */
    @Override
    public Action createPathMeep() {
        return actionBuilder
                .addPath(preload(5))
                .addPath(intake1(5))
                .addPath(place1())
                .addPath(intake2())
                .addPath(place2())
                .addPath(intake3())
                .addPath(pickup1())
                .addPath(cycle1(1))
                .addPath(pickup(1))
                .addPath(cycle(2))
                .addPath(pickup(2))
                .addPath(cycle(3))
                .addPath(pickup(3))
                .addPath(cycle(4))
                .addPath(basketIntake(4))
                .addPath(basketCycle1(0))
//                .addPath(basketIntake2(0))
                .build();
    }
}
