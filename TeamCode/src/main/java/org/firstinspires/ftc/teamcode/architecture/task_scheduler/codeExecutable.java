package org.firstinspires.ftc.teamcode.architecture.task_scheduler;

/**
 * codeExecutable is basically like Runnable, in fact idk why Matthew didn't use Runnable
    * I think I'm just slow, there's probably a reason but I'm too lazy to find out why
 */
@FunctionalInterface
public interface codeExecutable {
    void run() throws Exception;
}
