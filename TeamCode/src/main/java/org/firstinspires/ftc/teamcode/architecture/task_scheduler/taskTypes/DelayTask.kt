package org.firstinspires.ftc.teamcode.architecture.task_scheduler.taskTypes

import kotlinx.coroutines.delay
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Task

/**
 * Again, delay is a cancelable function w/ suspend
    * other commands can happen even while it's waiting for the delay to finish
 */
class DelayTask(private val delay: Long): Task() {
    override suspend fun execute() {
        delay(delay)
    }
}