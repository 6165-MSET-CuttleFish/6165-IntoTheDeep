package org.firstinspires.ftc.teamcode.sim;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;
import static org.firstinspires.ftc.teamcode.pathing.BasketPath.basketCycleEndTangent;
import static org.firstinspires.ftc.teamcode.pathing.BasketPath.basketCyclePose;
import static org.firstinspires.ftc.teamcode.pathing.BasketPath.cyclePose;
import static org.firstinspires.ftc.teamcode.roadrunner.Drive.PARAMS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.Robot;
import org.firstinspires.ftc.teamcode.architecture.control.AccelVelConstraint;
import org.firstinspires.ftc.teamcode.pathing.BasketPath;
import org.firstinspires.ftc.teamcode.pathing.Specimen5_1Path;
import org.firstinspires.ftc.teamcode.pathing.Specimen5_2Path;
import org.firstinspires.ftc.teamcode.pathing.SpecimenPath;
import org.opencv.core.Mat;

public class MeepMeepTesting {
    public static TrajectoryActionBuilder intakeCycle1(TrajectoryActionBuilder builder, double heading) {
        double x = basketCyclePose.x + (1/(Math.tan(Math.toRadians(heading))) * (cyclePose.position.y - 30 - basketCyclePose.y));

        return builder
                .strafeTo(new Vector2d(x, cyclePose.position.y - 30), AccelVelConstraint.VelConstraint(60), AccelVelConstraint.AccelConstraint(-60, 60))
                .splineToSplineHeading(new Pose2d(cyclePose.position, Math.toRadians(15)), Math.toRadians(0), AccelVelConstraint.VelConstraint(60), AccelVelConstraint.AccelConstraint(-60, 60));
    }

    public static TrajectoryActionBuilder intakeCycle(TrajectoryActionBuilder builder, double cycle, double heading) {
        double angleOffset = 0;

        if (cycle == 1) {
            angleOffset = 15;
        } else if (cycle == 2) {
            angleOffset = -15;
        }

        angleOffset = Math.toRadians(angleOffset);

        double x = basketCyclePose.x + (1/(Math.tan(Math.toRadians(heading))) * (cyclePose.position.y - 30 - basketCyclePose.y));

        return builder
                .strafeTo(new Vector2d(x, cyclePose.position.y - 30), AccelVelConstraint.VelConstraint(60), AccelVelConstraint.AccelConstraint(-60, 60))
                .splineToSplineHeading(new Pose2d(cyclePose.position, angleOffset), Math.toRadians(0), AccelVelConstraint.VelConstraint(60), AccelVelConstraint.AccelConstraint(-60, 60));
    }

    public static TrajectoryActionBuilder basketCycle(TrajectoryActionBuilder builder, double heading) {
        double x = basketCyclePose.x + (1/(Math.tan(Math.toRadians(heading))) * (cyclePose.position.y - 10 - basketCyclePose.y));

        return builder
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(x, cyclePose.position.y - 10, Math.toRadians(basketCycleEndTangent)), Math.toRadians(basketCycleEndTangent + 180), AccelVelConstraint.VelConstraint(30), AccelVelConstraint.AccelConstraint(-30, 30))
                .strafeTo(new Vector2d(basketCyclePose.x, basketCyclePose.y), AccelVelConstraint.VelConstraint(40), AccelVelConstraint.AccelConstraint(-40, 40));
    }


    public static void main(String[] args) {
        /** make sure it's null in case it's still initialized from a previous run */
        Robot.robot = null;

        MeepMeep meepMeep = new MeepMeep(800);

        SpecimenPath s = new SpecimenPath(Context.POSES.specimenStart);
        Specimen5_1Path s5_1 = new Specimen5_1Path(Context.POSES.specimen5_1Start);
        Specimen5_2Path s5_2 = new Specimen5_2Path(Context.POSES.specimen5_1Start);
        BasketPath b0_8 = new BasketPath(Context.POSES.basketStart);
        //BasketPath b = new BasketPath(Context.POSES.basketStart);

        RoadRunnerBotEntity sBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(PARAMS.maxWheelVel, PARAMS.maxProfileAccel, PARAMS.maxAngVel, PARAMS.maxAngAccel, 15)
                .build();
        RoadRunnerBotEntity s5_1Bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(PARAMS.maxWheelVel, PARAMS.maxProfileAccel, PARAMS.maxAngVel, PARAMS.maxAngAccel, 15)
                .build();

        RoadRunnerBotEntity s5_2Bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(PARAMS.maxWheelVel, PARAMS.maxProfileAccel, PARAMS.maxAngVel, PARAMS.maxAngAccel, 15)
                .build();
        RoadRunnerBotEntity b0_8Bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(PARAMS.maxWheelVel, PARAMS.maxProfileAccel, PARAMS.maxAngVel, PARAMS.maxAngAccel, 15)
                .build();

        RoadRunnerBotEntity b0_7Bot = new DefaultBotBuilder((meepMeep))
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(PARAMS.maxWheelVel, PARAMS.maxProfileAccel, PARAMS.maxAngVel, PARAMS.maxAngAccel, 15)
                .build();

        sBot.runAction(s.createPathMeep());
        s5_1Bot.runAction(s5_1.createPathMeep());
//        s5_2Bot.runAction(s5_2.createPathMeep());
//        b0_8Bot.runAction(b0_8.createPathMeep());

        double xCycle = cyclePose.position.x - 10;
        double yCycle = cyclePose.position.y - 2;
        double xDist = Math.abs(xCycle - basketCyclePose.x);
        double yDist = Math.abs(yCycle - basketCyclePose.y);
        double angle = Math.atan2(yDist, xDist);

        double xAfterCont = basketCyclePose.x - 2;
        double yAfterCont = basketCyclePose.y + 4;

        double xAfterStrafe = cyclePose.position.x - 35;
        double yAfterStrafe = Math.tan(Math.toRadians(basketCycleEndTangent)) * (xAfterStrafe - xAfterCont) + yAfterCont;

        double angleOffset = 0;


        TrajectoryActionBuilder b = b0_7Bot.getDrive().actionBuilder(cyclePose)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(xCycle, yCycle, angle), angle + Math.toRadians(180), AccelVelConstraint.VelConstraint(60), AccelVelConstraint.AccelConstraint(-50, 60))
                .strafeToConstantHeading(new Vector2d(basketCyclePose.x, basketCyclePose.y), AccelVelConstraint.VelConstraint(55), AccelVelConstraint.AccelConstraint(-40, 50))
                .splineToConstantHeading(new Vector2d(xAfterCont, yAfterCont), angle,  AccelVelConstraint.VelConstraint(10), AccelVelConstraint.AccelConstraint(-10, 10))
                //.strafeToSplineHeading(new Vector2d(xAfterStrafe, yAfterStrafe), Math.toRadians(basketCycleEndTangent), AccelVelConstraint.VelConstraint(55), AccelVelConstraint.AccelConstraint(-40, 50))
                .splineToSplineHeading(new Pose2d(cyclePose.position.x, cyclePose.position.y, angleOffset), angleOffset);

        b0_7Bot.runAction(b.build());
        /**
         * currently cannot set a background because it takes a java.awt.Image which is not available in android module
            * :( disadvantage of putting meepmeep in TeamCode
         * so I cloned MeepMeep, changed the default image to IntoTheDeep field in the clone, and send it to here via jitpack
            * no way jitpack worked like first try WUTTTT
            * if someone wants to make Image an android-friendly format in MeepMeep, ENJOY :)
            * the repository is on cuttle organization
         */
        meepMeep
                .setDarkMode(false)
                .setBackgroundAlpha(1f)
                .addEntity(s5_1Bot)
                .start();
    }
}