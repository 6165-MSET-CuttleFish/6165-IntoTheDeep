package org.firstinspires.ftc.teamcode.architecture.task_scheduler

import android.os.SystemClock
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.async
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.architecture.Robot.robot
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.taskTypes.AwaitTask
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.taskTypes.DelayTask

/**
 * GlobalScope.launch() is non-blocking (asynchronous)
    * this is what we use mainly, it launches a coroutine that runs in the background WHILE the main opMode runs
    * Coroutines close automatically when done which is very convenient
 * runBlocking is, of course, blocking (synchronous)
    * The entire POINT of task scheduler is to be non-blocking, to run task logic in the background while updating hardware elsewhere
        * Therefore, there is basically 0 use case for this
    * What people usually do is put their entire code in a runBlocking {} and inside put asynchronous coroutines
        * We don't do this
 */
class TaskScheduler {

    fun scheduleTaskList(t: List<Task>) {
        if(robot.opMode!!.opModeIsActive()) {
            val taskList: List<Task> = t
            GlobalScope.launch(Dispatchers.Default) {
                for(task in taskList) {
                    val job = async(Dispatchers.Default) {
                        task.execute()
                    }
                    if(task.javaClass == AwaitTask::class.java || task.javaClass == DelayTask::class.java) {
                        job.join()
                    }
                }
            }
        }
    }

    fun scheduleTask(t: Task): Unit {
        GlobalScope.launch(Dispatchers.Default) {
            t.execute()
        }
    }

    fun scheduleTaskListBlocking(t: List<Task>): Unit {
        val taskList: List<Task> = t
        runBlocking {
            for (task in taskList) {
                val job = async(Dispatchers.Default) {
                    task.execute()
                }
                if (task.javaClass == AwaitTask::class.java || task.javaClass == DelayTask::class.java) {
                    job.join()
                }
            }
        }
    }

    fun scheduleTaskListBlocking(t: List<Task>, timeout: Long): Unit {
        val startTime: Long = SystemClock.elapsedRealtime()
        val taskList: List<Task> = t
        val coroutineJob = GlobalScope.launch(Dispatchers.Default) {
            for (task in taskList) {
                if (SystemClock.elapsedRealtime() - startTime > timeout)
                    break

                val job = async(Dispatchers.Default) {
                    task.execute()
                }
                if (task.javaClass == AwaitTask::class.java || task.javaClass == DelayTask::class.java) {
                    while (!job.isCompleted) {
                        if (!robot.opMode!!.opModeIsActive())
                            job.cancel()
                    }
                }
            }
        }

        while (!coroutineJob.isCompleted) {
            if (SystemClock.elapsedRealtime() - startTime > timeout || !robot.opMode!!.opModeIsActive())
                coroutineJob.cancel()
        }
    }
}