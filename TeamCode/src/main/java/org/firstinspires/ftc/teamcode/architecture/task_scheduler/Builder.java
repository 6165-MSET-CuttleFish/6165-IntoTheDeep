package org.firstinspires.ftc.teamcode.architecture.task_scheduler;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.arcrobotics.ftclib.gamepad.ButtonReader;

import org.firstinspires.ftc.teamcode.architecture.modules.moduleUtil.Module;
import org.firstinspires.ftc.teamcode.architecture.modules.moduleUtil.ModuleState;
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.taskTypes.AwaitTask;
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.taskTypes.DelayTask;
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.taskTypes.ExecutionTask;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;

/**
 * Great place to learn how builders work
 * Basically the goal is to find a way to add a bunch of items neatly and use them to return some object or value
    * e.g., builder.task1.task2.task3.task4.task5.build() to take in some information about the 5 tasks and turn it into something new
    * It's very easy to do this, you just return the same builder with the new task added onto a list
    * in other words, you add the element to the list, and you return itself, the same builder object
    * now you are where you started, but your list has the added element in it. PROGRESS!!
 * In this builder, our list is called tasks, and it's a list of type Task
 * Then we want to find some information related to the resultant tasks in our build() function
    * this is basically compiling the list you created and returning the type you want
    * In this case, we literally just care about the list of tasks to run
    * so in build(), we just return tasks, the list we generated in the builder
 * in other builders (see ActionBuilder), we might combine the elements or otherwise interpret the list differently
 * it's basically just putting things in a list and interpreting it, but with better readability and sometimes better functionality
    * for example, it's very convenient to have a lastModuleCalled and lastModuleStateCalled value
        * if you put everything in a list at once rather than sequentially in a builder, you wouldn't have this functionality
 */
public class Builder {

    List<Task> tasks = new ArrayList<>();
    Module lastModuleCalled;
    ModuleState lastModuleStateCalled;

    public static Builder create() {
        return new Builder();
    }

    public Builder executeCode(codeExecutable task) {
        tasks.add(new ExecutionTask(task));
        return this;
    }
    public Builder moduleAction(Module m, ModuleState s) {
        tasks.add(new ExecutionTask(() -> m.setState(s)));
        lastModuleCalled = m;
        lastModuleStateCalled = s;
        return this;
    }

    public Builder await(Callable<Boolean> runCondition) {
        tasks.add(new AwaitTask(runCondition));
        return this;
    }

    public Builder delay(long delay) {
        tasks.add(new DelayTask(delay));
        return this;
    }
    public Builder addTaskList(List<Task> taskList) {
        List<Task> tasksToAdd = new ArrayList<>(taskList);
        tasks.addAll(tasksToAdd);
        return this;
    }

    /** At the time of the task execution, it checks if the boolean is true or false */
    public Builder conditionalModuleAction(Module m, ModuleState s, Callable<Boolean> conditional) {
        tasks.add(new ExecutionTask(() -> {
            if (conditional.call()) {
                m.setState(s);
            }
        }));
        lastModuleCalled = m;
        lastModuleStateCalled = s;
        return this;
    }
    public Builder addTask(Task t) {
        tasks.add(t);
        return this;
    }

    public Builder awaitButtonPress(ButtonReader b) {
        tasks.add(new AwaitTask(b::wasJustPressed));
        return this;
    }

    public Builder awaitDtXWithin(double x, double threshold) {
        tasks.add(new AwaitTask(() -> Math.abs(robot.pose.position.x - x) < threshold));
        return this;
    }

    public Builder awaitDtYWithin(double y, double threshold) {
        tasks.add(new AwaitTask(() -> Math.abs(robot.pose.position.y - y) < threshold));
        return this;
    }
    public Builder awaitPreviousModuleActionCompletion() {
        tasks.add(new AwaitTask(() -> !lastModuleCalled.isBusy()));
        return this;
    }


    public Builder insertTask(int index, Task t) {
        tasks.add(index, t);
        return this;
    }

    /**
     * as mentioned above, it takes everything the builder put in and returns something using that data
     * in this case, it's literally just that same list you built
     */
    public List<Task> build() {
        return tasks;
    }
}
