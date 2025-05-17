package org.firstinspires.ftc.teamcode.architecture.task_scheduler.taskTypes

import kotlinx.coroutines.delay
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Task
import java.util.concurrent.Callable

/**
 * A Callable is basically a Runnable, but it can return a value (e.g. a boolean)
    * This boolean is what AwaitTask waits for
 * Just a note, execute is also an abstract method of Task
    * see Marker to understand how abstract methods work and why they're useful
 */

class AwaitTask @JvmOverloads constructor(private val condition: Callable<Boolean>, private val loopTime: Long = 10): Task() {
    override suspend fun execute() {
        while(!condition.call()) {
            delay(loopTime)
        }
    }
}