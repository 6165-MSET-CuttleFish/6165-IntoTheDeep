package org.firstinspires.ftc.teamcode.pathing.path_architecture;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.architecture.markers.Marker;
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Task;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.atomic.AtomicReference;

/**
 * another builder, used to make the entire auto sequence
 * there are essentially 4 components in an auto
    * 1. the drivetrain path: this is done through .addPath() and is passed as a TrajectoryActionBuilder (RR's own path builder)
        * we're using their builders as components in our builder
    * 2. the markers: this is also done through .addPath() and is passed as a Marker... m
        * The method treats Marker... m the exact same way as Marker[] m
        * The difference is that you can pass in a bunch of individual elements rather than just passing in an array
            * It takes all those individual markers and makes an array out of them
    * 3. the actions: this is done through .addAction()
        * you can pass in a Runnable or a List<Task>
            * DO NOT DO .addAction(() -> actions.action())
            * Instead do either:
                * .addAction(() -> robot.scheduler.scheduleTaskList(actions.action()))
                * or the shorthand: .addAction(actions.action())
    * 4. the delays: this is done through .addDelayMs()
        * you pass in the time in milliseconds to wait
        * this is also non-blocking, just as Matthew's delays are non-blocking

 */
public class ActionBuilder {
    public ArrayList<Action> actions;

    public static Pose2d pose = new Pose2d(0,0,0);
    public ActionBuilder() {
        actions = new ArrayList<>();
    }


    public ActionBuilder addPath(TrajectoryActionBuilder a, Marker... m) {

        /**
         * why parallel action? why not just add them one by one?
            * if you put markers first, it'll detect that isBusy is false (cause path hasn't started yet)
                * it will run instantaneously (because it thinks it finished path and the markers were just not satisfied)
            * if you put path first, it'll finish the path before adding the markers, defeating the purpose
                * everything is in a SequentialAction, so it'll wait for the path to finish before moving onto the next action in the list
            * the solution is to run the path RIGHT BEFORE the markers (basically at the same time, but bc a.build() comes before addMarkers(), isBusy will be true when the markers run)
         */


        actions.add(new ParallelAction(
                a.build(),
                new addMarkers(m)
        ));


        return this;
    }

    public ActionBuilder addWeirdPath(TrajectoryActionBuilder a, Callable<Pose2d> pose, Marker... m) {
        actions.add(new ParallelAction(
                a.build(),
                new addMarkers(m)
        ));


        return this;
    }
    public ActionBuilder addDelayMs(double time) {
        // implement
        actions.add(new SleepAction(time/1000.0));
        return this;
    }

    public ActionBuilder addAction(Action willisa) {
        actions.add(willisa);
        return this;
    }

    public ActionBuilder addAction(Runnable r) {
        actions.add(new actionMaker(r));
        return this;
    }
    public ActionBuilder addAction(List<Task> r) {
        actions.add(new actionMaker(() -> robot.scheduler.scheduleTaskList(r)));
        return this;
    }

    public ActionBuilder addParallelActions(Action... actions) {
        this.actions.add(new ParallelAction(actions));
        return this;
    }



    public ActionBuilder addAwait(Callable<Boolean> bool) {
        actions.add(new awaitAction((bool)));
        return this;
    }

    public ActionBuilder addAwaitTimeout(Callable<Boolean> bool, int ms) {
        ElapsedTime timer = new ElapsedTime();
        actions.add(new awaitAction(() -> {
            try {
                return bool.call() || timer.milliseconds() > ms;
            } catch (Exception e) {
                e.printStackTrace();
                return true;
            }
        }));
        return this;
    }


    public Action build() {
        return new SequentialAction(actions);
    }

    /**
     * Why is this necessary? Why can't we just run this line of code in addPath()?
        * The issue is it would just run right when the path is initialized, not during the actual path it's set in
            * It's the same reason we need Callable<Boolean> in TaskScheduler rather than just passing in a Boolean
            * this action class runs it in sequence, and immediately becomes false
     */
    static class addMarkers implements Action {
        Marker[] markers;
        ElapsedTime wait;
        boolean runOnce = true;
        public addMarkers(Marker[] m) {
            this.markers = m;
            wait = new ElapsedTime();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (runOnce) {
                wait.reset();
                runOnce = false;
            }

            if (robot != null) {

                /** wait a little to ensure path is started (though it should be sequential, idk how Actions multithreading works */
                if (wait.milliseconds() > 30) {
                    robot.markerList.addAll(Arrays.asList(markers));
                    return false;
                }
            }
            return true;
        }

    }

    public static class actionMaker implements Action {
        Runnable r;
        public actionMaker(Runnable r) {
            this.r = r;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            r.run();
            return false;
        }
    }

    public static class setPose implements Action {
        Callable<Pose2d> p;
        public setPose(Callable<Pose2d> p) {
            this.p = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            try {
                pose = p.call();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
            return false;
        }
    }

    public static class awaitAction implements Action {
        Callable<Boolean> c;
        public awaitAction(Callable<Boolean> c) {
            this.c = c;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            try {
                return (!c.call());
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }
    public static class conditionalAction implements Action {
        Callable<Boolean> c;
        Action a;
        public conditionalAction(Callable<Boolean> c, Action a) {
            this.c = c;
            this.a = a;
        }
        public conditionalAction(Callable<Boolean> c, List<Task> t) {
            this.c = c;
            this.a = new InstantAction(() -> robot.runAction(t));
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            try {
                if (c.call() == false) {
                    return false;
                }
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
            return a.run(telemetryPacket);
        }
    }
}

