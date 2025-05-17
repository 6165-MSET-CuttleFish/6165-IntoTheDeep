package org.firstinspires.ftc.teamcode.pathing.path_architecture;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;
import static org.firstinspires.ftc.teamcode.roadrunner.Drive.PARAMS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.roadrunner.Drive;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class Path {
    public ActionBuilder actionBuilder;
    public static TrajectoryActionBuilder builder;

    public ArrayList<Action> list;

    /**
     * what is this weird MeepMeep + Robot hybrid bs you might ask?
        * good question, idk wtf im doing lol
     * Here's the problem and I'm hoping yall have ideas cause I don't
        * basically I wanted to create a createPath() function which returns the main action for both MeepMeep and auto
        * However, I also wanted to put markers and RobotActions in the same place I put paths
        * It will throw an error in MeepMeep if it tries to run anything related to Robot (robot does not exist in MeepMeep)
        * I tried checking if robot is null in addPath() and addAction() but the problem is java throws the error READING THE PARAMETER, not actually calling it
            * you would need to check if robot is null outside of the function, and the problem with that is you can't take JUST the path and NOT the markers or actions
        * I tried creating a MockRobot class, but anything that extends Robot must take an OpMode object or something of the like (HardwareMap, etc, etc)
            * MeepMeep doesn't have an OpMode, that's the fundamental problem
     * So I settled on this annoying solution
        * a path returns two sequences, one for robot and one for meep (createPath() and createPathMeep(), respectively)
            * meep is a copy-paste of robot path but without markers and actions
                * to update meep, copy paste the robot path in and delete actions and markers
                * they both use same paths so robot is updated automatically if u change path for meep
     * Long-term solutions
        * 1. keep this
        * 2. access the TrajectoryBuilders directly (imo that's even worse)
        * 3. use ur big brains to find a cheap scuffed way around this solution
        * 4. maybe there's a way to pass a potentially null object as parameter? Kotlin has optionals, but idk if parameters just become null if a reference within it is null
     * _____________________________________________________________________________________________________
     * Either initialize path builder (this is RR's path builder, NOT my auto builder) with meepmeep or with robot
        * It checks if robot == null, if it is it does MeepMeep
            * if u specimens decide to run paths without robot it's ur fault if it breaks lmfaooo
     * pose of the robot is set in initialization by default
        * you can set it again onStart() in the opMode so you can move the robot during init() (I put it in SpecimenAuto and BasketAuto)
     */
    public Path(Pose2d startPose) {

        if (robot == null) {
            builder = new DefaultBotBuilder(new MeepMeep(800))
                    .setDimensions(14, 17)
                    .setConstraints(PARAMS.maxWheelVel, Drive.PARAMS.maxProfileAccel, Drive.PARAMS.maxAngVel, Drive.PARAMS.maxAngAccel, 15)
                    .build()
                    .getDrive()

                    .actionBuilder(startPose);
        } else {
            builder = robot.actionBuilder(startPose);
            robot.pose = startPose;
        }


        list = new ArrayList<>();
        actionBuilder = new ActionBuilder();
    }
    /** robot auto sequence */
    public abstract Action createPath();

    /** meepmeep sequence */
    public abstract Action createPathMeep();

    public void set(ActionBuilder b) {
        actionBuilder = b;
    }
}
