package org.firstinspires.ftc.teamcode.architecture.markers;

import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Task;

import java.util.List;

/**
 * Was too lazy to make an instant marker, so it just uses a temporal marker with 0.0001 delay lol
 */
public class InstantMarker extends TemporalMarker {
    public InstantMarker(Runnable action) {
        super(0.0001, action);
    }
    public InstantMarker(List<Task> action) {
        super(0.0001, action);
    }
}