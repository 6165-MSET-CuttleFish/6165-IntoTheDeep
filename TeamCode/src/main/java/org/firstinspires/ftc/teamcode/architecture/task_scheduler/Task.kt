package org.firstinspires.ftc.teamcode.architecture.task_scheduler

/**
 * suspend means exactly what it says, it can stop and resume at a later time
 * imagine a task is in the middle of running (say it's an await so it's in a while loop)
    * and another taskList in a different coroutine tries to run
        * suspend is required to make it non-blocking (that is to say, it's required to let the other task list run and then go back to the while loop)
 */
abstract class Task {
    abstract suspend fun execute();
}