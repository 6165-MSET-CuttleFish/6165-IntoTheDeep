package org.firstinspires.ftc.teamcode.architecture.markers;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Task;

import java.util.List;

public class SpatialMarker extends Marker {

    /**
     * Activates the action once it reaches the coord
     */
    private final double coord;

    /**
     * Which axis the coord lies on
     */
    private final Axis axis;

    /**
     * Used when the first .run() is run, in this case used to find forward
     */
    boolean runOnce = true;

    /**
     * If inFrontOfCoord is true, the action runs once pos < coord. If false, the action runs once pos > coord
        * forward is determined based on if the robot starts in front of or behind the coord
            * The marker is satisfied once the robot reaches the opposite side of the coord from where it started
     */
    boolean inFrontOfCoord = false;

    public SpatialMarker(double coord, Axis axis, Runnable action) {
        this.coord = coord;
        this.axis = axis;
        this.action = action;
    }
    public SpatialMarker(double coord, Axis axis, List<Task> action) {
        this.coord = coord;
        this.axis = axis;
        this.action = () -> robot.scheduler.scheduleTaskList(action);
    }

    @Override
    public boolean run() {
        boolean markerActive = true;

        if (runOnce) {
            if (axis.equals(Axis.X)) {
                inFrontOfCoord = robot.pose.position.x > coord;
            } else if (axis.equals(Axis.Y)) {
                inFrontOfCoord = robot.pose.position.y > coord;
            }
            runOnce = false; // sets it to false so it only runs once
        }

        if (axis.equals(Axis.X)) {
            if (inFrontOfCoord) {
                markerActive = robot.pose.position.x > coord;
            } else {
                markerActive = robot.pose.position.x < coord;
            }
        } else if (axis.equals(Axis.Y)) {
            if (inFrontOfCoord) {
                markerActive = robot.pose.position.y > coord;
            } else {
                markerActive = robot.pose.position.y < coord;
            }
        } else if (axis.equals(Axis.ANGLE)) {
            double angleDifference = toDegrees(robot.pose.heading.toDouble() - coord) % 360;
            markerActive = angleDifference > 5;
        }

        if (!markerActive && action != null) {
            action.run(); // Execute the lambda function
        }

        return markerActive;
    }
}