package org.firstinspires.ftc.teamcode.architecture.task_scheduler.taskTypes

import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Task
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.codeExecutable

/**
 * a codeExecutable is essentially a Runnable, in fact idk why Matthew didn't use Runnable
    * there must be a fancy reason that's beyond the scope of puny brain
 * this one method is abstracted to do all instantaneous actions (e.g. module setting, logging, etc)
 */
class ExecutionTask (private val codeExecutable: codeExecutable): Task() {
    override suspend fun execute() {
        codeExecutable.run()
    }
}