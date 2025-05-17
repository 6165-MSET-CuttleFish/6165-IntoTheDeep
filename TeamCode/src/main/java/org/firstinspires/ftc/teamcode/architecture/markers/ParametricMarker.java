package org.firstinspires.ftc.teamcode.architecture.markers;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Task;

import java.util.List;

public class ParametricMarker extends Marker {

    /**
     * position from 0 to 1
     */
    private final double parametricTime;
    private double time;

    /**
     * used to check if timer in seconds exceeds Marker time
     */
    private ElapsedTime timer;

    /**
     * same purpose as in SpatialMarker
     */
    private boolean runOnce = false;
    public ParametricMarker(double parametricTime, Runnable action) {
        this.parametricTime = parametricTime;
        this.action = action;
    }
    public ParametricMarker(double parametricTime, List<Task> action) {
        this.parametricTime = parametricTime;
        this.action = () -> robot.scheduler.scheduleTaskList(action);
    }

    // TEMPORAL WORK
    @Override
    public boolean run() {
        if (!runOnce) {
            timer = new ElapsedTime();
            timer.reset();
            runOnce = true;

            if (robot.t != null) {
                time = parametricTime * robot.t.duration;
            } else {
                throw new ArithmeticException("parametric marker run without a trajectory!");
            }
        }
        if (timer.seconds() > time) {
            if (action != null) {
                action.run(); // Execute the lambda function
            }
            return false;
        }
        return true;
    }
}