package org.firstinspires.ftc.teamcode.architecture.markers;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Task;

import java.util.List;

public class TemporalMarker extends Marker {

    /**
     * time to reach in seconds
     */
    private final double time;

    /**
     * used to check if timer in seconds exceeds Marker time
     */
    private ElapsedTime timer;

    /**
     * same purpose as in SpatialMarker
     */
    private boolean runOnce = false;
    public TemporalMarker(double time, Runnable action) {
        this.time = time;
        this.action = action;
    }
    public TemporalMarker(double time, List<Task> action) {
        this.time = time;
        this.action = () -> robot.scheduler.scheduleTaskList(action);
    }

    // TEMPORAL WORK
    @Override
    public boolean run() {
        if (!runOnce) {
            timer = new ElapsedTime();
            timer.reset();
            runOnce = true;
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