package org.firstinspires.ftc.teamcode.architecture.control;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;
import static org.firstinspires.ftc.teamcode.roadrunner.Drive.PARAMS;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;

import org.firstinspires.ftc.teamcode.roadrunner.Drive;

import java.util.Arrays;
import java.util.List;

/**
 * I think this class is pretty self-explanatory, you can make more functions as you see fit
 * The capabilities for acceleration control are quite limited
    * it's prob just a nightmare to make motion profiles with varying acceleration, but maybe there is a way? idk
 * TranslationalVelConstraint refers to ALL DIRECTIONS OF TRAVEL, NOT JUST PERPENDICULAR TO PATH
 * https://rr.brott.dev/docs/v1-0/guides/variable-constraints/
 */
public class AccelVelConstraint {
    public static VelConstraint VelConstraint(double vel, double angularVel) {
        if (robot != null)  {
            return new MinVelConstraint(Arrays.asList(
                    robot.kinematics.new WheelVelConstraint(vel),
                    new AngularVelConstraint(angularVel)
            ));
        }
        return new MinVelConstraint(Arrays.asList(
                new AngularVelConstraint(angularVel)
        ));
    }
    public static VelConstraint VelConstraint(double vel) {
        if (robot != null)  {
            return new MinVelConstraint(Arrays.asList(
                    new TranslationalVelConstraint(vel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
        }
        return new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(vel),
                new AngularVelConstraint(PARAMS.maxAngVel)
        ));
    }
    public static AccelConstraint AccelConstraint(double decel, double accel) {
        if (decel > 0) {
            decel *= -1;
        }
        return new ProfileAccelConstraint(decel, accel);
    }
}