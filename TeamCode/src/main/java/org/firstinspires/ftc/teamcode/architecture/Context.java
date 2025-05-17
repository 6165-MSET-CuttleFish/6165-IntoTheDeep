package org.firstinspires.ftc.teamcode.architecture;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.tele.ACTION_MODE;
import org.firstinspires.ftc.teamcode.architecture.tele.DRIVE_TYPE;
import org.firstinspires.ftc.teamcode.vision.Pipelines.Color;

/**
 * got rid of isAuto, it makes things cluttered and honestly there's no use for it
 * the only instance remaining was RobotActions, and I just passed an isAutoDrive parameter in the taskList instead
 */

@Config
public final class Context {

    /**
     * this was used to switch between float (for DT motors to coast) and brake (for DT motors to halt) at 0 power
     * it's probably unnecessary to still have this now that we don't tip anymore
     * I kept it just in case but commented out the actual zeroPower stuff (see Tele)
     */
    public static boolean isFloat = false, read = true, isColorSensor = false, isLimelight = false, readyForTapeMeasure = false;
    /**
     * used to store if robot is on red or blue side, there's no implementation to set it currently
     */
    public static Color color = Color.RED;
    public static int intakeDriveTime = 200, depositDriveTime = 200, specimenDriveTime = 100, depositWaitTime = 0;
    public static int hangSequenceCount = 0;
    public static double pitchSpeed = 0.01, ninja = 1, ninjaStrafe = 1;
    public static ACTION_MODE actionMode = ACTION_MODE.INTAKE;
    public static DRIVE_TYPE type = DRIVE_TYPE.MANUAL;
    public static ElapsedTime autoDriveTimer = new ElapsedTime();
    public static int manualLoopTimeDelay = 0;

    public static class PIVOT_EXTENSION_RESET {
        public static boolean hasReset = false;
        public static double pivotOffset;
    }

    /**
     * Pose is reference to (0,0) being in the center of field, 0 heading pointing to the right, and the red basket being in bottom left
     * Makes no sense? run MeepMeepTesting to see how it's mapped
     */
    public static class POSES {
        public static Pose2d basketStart = new Pose2d(-36, -65, Math.toRadians(0));
        public static Pose2d specimenStart = new Pose2d(10.8547, -57.003, Math.toRadians(-90));
        public static Pose2d specimen5_1Start = new Pose2d(10.8547, -57.003, Math.toRadians(90));
    }
    public static class AUTO_SELECTOR {
        /**
         * in milliseconds
         */
        public static long waitTime = 0;
    }

    public static void setType(DRIVE_TYPE type) {
        Context.type = type;
        autoDriveTimer.reset();
    }
    public static double RESET_DELAY_MS = 3000;
}
