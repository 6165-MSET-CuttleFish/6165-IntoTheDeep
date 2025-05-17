package org.firstinspires.ftc.teamcode.pathing;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;
import static org.firstinspires.ftc.teamcode.roadrunner.Drive.PARAMS;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.control.AccelVelConstraint;
import org.firstinspires.ftc.teamcode.architecture.markers.Axis;
import org.firstinspires.ftc.teamcode.architecture.markers.SpatialMarker;
import org.firstinspires.ftc.teamcode.architecture.markers.TemporalMarker;
import org.firstinspires.ftc.teamcode.architecture.modules.PivotExtension;
import org.firstinspires.ftc.teamcode.pathing.path_architecture.Path;

@Config
public class SpecimenPath extends Path {
    public static double offset = 1;
    Vector2d scoreVector = new Vector2d(10, -31.5);
    Vector2d pickupVector = new Vector2d(38.5, -49);
    double angle = Math.toRadians(30);

    /**
     * use a little trig to ensure the line is straight approaching the sample
     */
    double xOffsetFromPickupSpline = -4;
    double yOffsetFromPickupSpline = xOffsetFromPickupSpline * tan(angle);

    double yPosSave = 0;

    ElapsedTime timer = new ElapsedTime();

    public SpecimenPath(Pose2d startPose) {
        super(startPose);
    }


    /**
     * robot path w/ delays, markers, actions, paths
     */
    @Override
    public Action createPath() {
        actionBuilder = actionBuilder
                .addAction(robot.actions.specimenSetupPreload())
                .addPath(preload(offset * 5)
//                        new SpatialMarker(scoreVector.y - 3, Axis.Y, robot.actions.specimenPreScore())
                )
                .addAction(robot.actions.specimenPreScore())
                .addAction(robot.actions.specimenScorePreload())
                     //   new SpatialMarker(scoreVector.y - 2, Axis.Y, robot.actions. specimenScoreAuto()))
                //.addAwait(isScored)

                .addPath(intake1(offset * 5),
                        new TemporalMarker(0.3, robot.actions.firstExtendAndActivateIntake())
                )
                .addAction(robot.actions.autoIntakeResetAndExtend())

                .addPath(place1(),
                        new TemporalMarker(0.08, robot.actions.extendAndActivateExtake(400))
                )

                .addPath(intake2(),
                        new TemporalMarker(0.2, robot.actions.ActivateIntake())
                )
                .addAction(robot.actions.autoIntakeResetAndExtend())

                .addPath(place2(),
                        new TemporalMarker(0.1, robot.actions.extendAndActivateExtake(200))
                )
                .addPath(intake3(),
                        new TemporalMarker(0.2, robot.actions.ActivateIntake())
                )
                .addAction(robot.actions.autoIntakeReset())

                .addPath(pickup1(),
                        new TemporalMarker(0.5, robot.actions.specimenGoToPickupYeet())
//                        new SpatialMarker(pickupVector.y + 2.5, Axis.Y, robot.actions.specimenPickup())
                        // pickupMarker
                )
                .addAction(robot.actions.specimenPickupTele())
                .addDelayMs(500);
        /*semicolon*/;
        for (int i = 1; i < 5; i++) {
            if (i == 1) {
                actionBuilder = actionBuilder
                        .addPath(cycle1(offset * i),
                                new SpatialMarker(scoreVector.y - 3, Axis.Y, robot.actions.specimenPreScore())

                        )
                        .addAction(robot.actions.specimenScore(true))
                /*the end*/;
            } else if (i == 4) {
            actionBuilder = actionBuilder
                    .addPath(cycle(offset * i),
                            new SpatialMarker(scoreVector.y - 3, Axis.Y, robot.actions.specimenPreScore())
                    )
                    .addAction(robot.actions.specimenScore(false))
            /*the end*/;
            } else {
                actionBuilder = actionBuilder
                        .addPath(cycle(offset * i),
                                new SpatialMarker(scoreVector.y - 3, Axis.Y, robot.actions.specimenPreScore())
                        )
                        .addAction(robot.actions.specimenScore(true))
                /*the end*/;
            }
            if (i != 4) {
                actionBuilder = actionBuilder
                        .addPath(pickup(offset * i),
                                new TemporalMarker(0.5, () -> robot.pivotExtension.setState(PivotExtension.ExtensionState.SPECIMEN_PICKUP_EXTENDED))
                        )
                        //.addDelayMs(300)
                        .addAction(robot.actions.specimenPickupTele())
                        .addDelayMs(250);
            } else {


                actionBuilder = actionBuilder
//                        .addAction(robot.actions.extendAndActivateIntakePark())
                        .addPath(park(offset * i)
                );
            }
        }
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
    public TrajectoryActionBuilder preload(double offset) {
        return builder
                .strafeToConstantHeading(new Vector2d(scoreVector.x - SpecimenPath.offset * 5 + offset, scoreVector.y), AccelVelConstraint.VelConstraint(60), AccelVelConstraint.AccelConstraint(-50, 55));
    }

    public TrajectoryActionBuilder intake1(double offset) {
        return preload(offset).endTrajectory().fresh()
                .setTangent(toRadians(-90))
                .lineToY(scoreVector.y - 5)
                .splineToSplineHeading(new Pose2d(44, -34.5, angle), angle, AccelVelConstraint.VelConstraint(60), AccelVelConstraint.AccelConstraint(-50, 50));

//                .splineToSplineHeading(new Pose2d(42 + xOffsetFromPickupSpline, -34.5 + yOffsetFromPickupSpline, angle), angle, AccelVelConstraint.VelConstraint(60), AccelVelConstraint.AccelConstraint(-50, 50))
//                .splineToConstantHeading(new Vector2d(44, -34.5), angle, AccelVelConstraint.VelConstraint(70), AccelVelConstraint.AccelConstraint(-50, 50));
    }

    public TrajectoryActionBuilder place1() {
        return intake1(5).endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(41, -41), Math.toRadians(-60));
    }

    public TrajectoryActionBuilder intake2() {
        return place1().endTrajectory().fresh()
                .setTangent(toRadians(45))
                .splineToSplineHeading(new Pose2d(51.5 + xOffsetFromPickupSpline, -35.5 + yOffsetFromPickupSpline, angle), angle)
                .splineToConstantHeading(new Vector2d(54, -34), angle);
    }

    public TrajectoryActionBuilder place2() {
        return intake2().endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(51, -34), Math.toRadians(-60));
    }

    public TrajectoryActionBuilder intake3() {
        return place2().endTrajectory().fresh()
                .setTangent(toRadians(45))
                .splineToSplineHeading(new Pose2d(60.5 + xOffsetFromPickupSpline, -36.5 + yOffsetFromPickupSpline, angle), angle)
                .splineToConstantHeading(new Vector2d(63, -34), toRadians(30));
    }

    public TrajectoryActionBuilder pickup1() {
        return intake3().endTrajectory().fresh()
                .setTangent(toRadians(180))
                .splineToSplineHeading(new Pose2d(pickupVector.x + 9.5, pickupVector.y + 5, toRadians(-90)), toRadians(-90))
                .splineToConstantHeading(new Vector2d(pickupVector.x + 9.5, pickupVector.y), toRadians(-90));
    }

    public TrajectoryActionBuilder cycle1(double offset) {
        return pickup1().endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(scoreVector.x - SpecimenPath.offset * 5 + offset + 3, scoreVector.y - 9), AccelVelConstraint.VelConstraint(PARAMS.maxWheelVel), AccelVelConstraint.AccelConstraint(-70, 70))
                .splineToConstantHeading(new Vector2d(scoreVector.x - SpecimenPath.offset * 5 + offset, scoreVector.y), Math.toRadians(90), AccelVelConstraint.VelConstraint(PARAMS.maxWheelVel), AccelVelConstraint.AccelConstraint(-70, 70));
    }

    public TrajectoryActionBuilder cycle(double offset) {

        return pickup(offset - SpecimenPath.offset).endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(scoreVector.x - SpecimenPath.offset * 5 + offset + 3, scoreVector.y - 9), AccelVelConstraint.VelConstraint(PARAMS.maxWheelVel), AccelVelConstraint.AccelConstraint(-100, 70))
                .splineToConstantHeading(new Vector2d(scoreVector.x - SpecimenPath.offset * 5 + offset, scoreVector.y), Math.toRadians(90), AccelVelConstraint.VelConstraint(PARAMS.maxWheelVel), AccelVelConstraint.AccelConstraint(-100, 70));

    }

    public TrajectoryActionBuilder pickup(double offset) {
        return cycle1(offset).endTrajectory().fresh()
                // .splineToConstantHeading(new Vector2d(scorePose.x - offset + 1, scorePose.y - 1), toRadians(-70), AccelVelConstraint.VelConstraint(vel), AccelVelConstraint.AccelConstraint(-decel, accel))
                .lineToYConstantHeading(scoreVector.y - 3, AccelVelConstraint.VelConstraint(70), AccelVelConstraint.AccelConstraint(-70, 70))
                .strafeToConstantHeading(pickupVector, AccelVelConstraint.VelConstraint(70), AccelVelConstraint.AccelConstraint(-60, 70));
    }
    public TrajectoryActionBuilder park(double offset) {
        return cycle(offset).endTrajectory().fresh()
                .setTangent(toRadians(-60))
                .strafeToSplineHeading(new Vector2d(35, -52), toRadians(-45), AccelVelConstraint.VelConstraint(70), AccelVelConstraint.AccelConstraint(-70, 70));

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
                .addPath(pickup(4))
                .addPath(park(4))
                .build();
    }


}