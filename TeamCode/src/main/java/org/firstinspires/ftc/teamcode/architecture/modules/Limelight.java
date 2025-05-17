package org.firstinspires.ftc.teamcode.architecture.modules;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;
import static org.firstinspires.ftc.teamcode.pathing.BasketPath.basketCycle;
import static org.firstinspires.ftc.teamcode.pathing.BasketPath.cyclePose;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.control.AccelVelConstraint;
import org.firstinspires.ftc.teamcode.architecture.modules.moduleUtil.Module;
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Builder;
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Task;
import org.firstinspires.ftc.teamcode.pathing.BasketPath;
import org.firstinspires.ftc.teamcode.vision.Pipelines.Color;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Manages Limelight vision processing for detecting and tracking objects.
 */
@Config
public class Limelight extends Module {

    // Telemetry and processing toggles
    public static boolean telemetryToggle = false;
    public static boolean displayTarget = true;
    public static boolean plotSamplesOnField = true;
    public static boolean useLimits = true;

    // Calibration constants
    public static double HORIZONTAL_OFFSET = -7.5;
    public static double VERTICAL_OFFSET = 24.1;

    // Distance and positioning constants
    public static double PRE_DISTANCE = 7;
    public static double PAST_DISTANCE = 7;
    public static double INTAKE_X = 12, INTAKE_Y = -2.5;
    public static double MIN_X_DISTANCE = 16;

    // Camera constants
    public static double FOV_X = 53.9;
    public static double FOV_Y = 42;
    public static double RES_X = 640;
    public static double RES_Y = 480;

    // State flags
    public static boolean isDoneIntakeAction = false;
    public static boolean isDoneSweeping = false;

    // Homography matrix for coordinate transformations
    private final double[][] homography = {
            {4.158013, 5.076947, -1146.668632},
            {-1.674058, 8.920382, -450.058258},
            {-0.000171, 0.003455, 1.000000}
    };

    public Limelight3A limelight;
    public double extensionLimelightTarget;
    private Sample target = new Sample(new Pose2d(0, 0, 0), 0);

    private static ArrayList<Sample> allSamples = new ArrayList<>();
    public ArrayList<Sample> lookahead = new ArrayList<>();


    /**
     * Initializes the Limelight module.
     * @param hardwareMap The hardware map to access the Limelight device
     */
    public Limelight(HardwareMap hardwareMap) {
        super(true, telemetryToggle);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        initializeLimelight();
        extensionLimelightTarget = PivotExtension.ExtensionState.RESET.getValue();
    }

    /**
     * Configures and starts the Limelight camera.
     */
    private void initializeLimelight() {
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(150);
        limelight.start();
    }

    // Module overrides
    @Override
    protected void write() {
        // Not used for this module
    }

    @Override
    protected void read() {
        if (Context.isLimelight) {
            if (!limelight.isRunning()) {
                limelight.start();
            }
            setTarget(processLimelightResults());
        } else {
            limelight.shutdown();
        }
    }

    @Override
    protected void initInternalStates() {
        // No internal state initialization needed
    }

    @Override
    protected void updateInternalStatus() {
        status = Status.IDLE;
    }

    @Override
    public void telemetryUpdate() {
        if (telemetryToggle) {
            super.telemetryUpdate();
            if (displayTarget) {
                displayTarget();
            }
            robot.tel.addLine();
            robot.tel.addData("Limelight isRunning", limelight.isRunning());
            robot.tel.addData("Limelight isConnected", limelight.isConnected());
        }
    }

    /**
     * Processes the latest results from the Limelight camera.
     * @return A Sample object representing the detected target
     */
    private Sample processLimelightResults() {
        LLResult results = limelight.getLatestResult();
        if (results != null && results.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = results.getDetectorResults();
            if (!detectorResults.isEmpty()) {
                return processDetectorResults(detectorResults);
            }
        }
        return new Sample(16, 0, -1);
    }

    /**
     * Processes detector results and filters targets based on priority and alliance.
     * @param detectorResults The list of detector results from Limelight
     * @return The highest priority sample detected
     */
    private Sample processDetectorResults(List<LLResultTypes.DetectorResult> detectorResults) {
        ArrayList<Sample> allianceSamples = new ArrayList<>();
        ArrayList<Sample> opposingSamples = new ArrayList<>();
        ArrayList<Sample> neutralSamples = new ArrayList<>();
        ArrayList<Sample> rejectedSamples = new ArrayList<>();


        // Process each detector result
        for (LLResultTypes.DetectorResult result : detectorResults) {
            Pose2d robotCoordinates = calculateRobotCoordinates(result);
            int id = result.getClassId();

            if (id == 2) { // Neutral
                if (withinRange(robotCoordinates)) {
                    neutralSamples.add(new Sample(robotCoordinates, id, result.getConfidence()));
                } else {
                    rejectedSamples.add(new Sample(robotCoordinates, id, result.getConfidence()));
                }
            } else if (id == Context.color.value) { // Alliance
                if (withinRange(robotCoordinates)) {
                    allianceSamples.add(new Sample(robotCoordinates, id, result.getConfidence()));
                } else {
                    rejectedSamples.add(new Sample(robotCoordinates, id, result.getConfidence()));
                }
            } else { // Opposing
                opposingSamples.add(new Sample(robotCoordinates, id, result.getConfidence()));
            }
        }

        if (plotSamplesOnField) {
            allSamples.clear();
            allSamples.addAll(allianceSamples);
            allSamples.addAll(opposingSamples);
            allSamples.addAll(neutralSamples);
            allSamples.addAll(rejectedSamples);
            for (Sample s : allSamples) {
                s.calculateGlobalPos();
            }
        }




        // Include neutral samples with alliance
        allianceSamples.addAll(neutralSamples);


        if (!allianceSamples.isEmpty()) {
            for (Sample sample : allianceSamples) {
                // Precompute priority for sorting and telemetry
                sample.checkAllPriority(opposingSamples, allianceSamples, rejectedSamples);
                if (telemetryToggle) robot.tel.addData("Priority: " + sample.getType(), sample.getMaxPriority());
            }

            allianceSamples.sort(Comparator.comparingDouble(Sample::getMaxPriority));
            lookahead = allianceSamples;
            lookahead.forEach(Sample::calculateGlobalPos);

            return allianceSamples.get(0); // Return the best (lowest priority) sample

        }


        return new Sample(0, 0, -1);
    }

    /**
     * Transforms detector result using homography matrix to get robot-relative coordinates.
     * @param result The detector result to transform
     * @return Robot-relative coordinates of the detected object
     */
    private Pose2d calculateRobotCoordinates(LLResultTypes.DetectorResult result) {
        double thetaX = result.getTargetXDegrees();
        double thetaY = -result.getTargetYDegrees();

        // Convert angles to pixel coordinates
        double x = (thetaX / FOV_X) * RES_X + RES_X / 2.0;
        double y = (thetaY / FOV_Y) * RES_Y + RES_Y / 2.0;

        // Apply homography transformation
        double X_prime = homography[0][0] * x + homography[0][1] * y + homography[0][2];
        double Y_prime = homography[1][0] * x + homography[1][1] * y + homography[1][2];
        double W = homography[2][0] * x + homography[2][1] * y + homography[2][2];

        // Convert to robot coordinate system (96 pixels/inch)
        double x_robot = -Y_prime / W / 96.0 + VERTICAL_OFFSET;
        double y_robot = X_prime / W / 96.0 + HORIZONTAL_OFFSET;
        return new Pose2d(x_robot, y_robot, 0);
    }

    /**
     * Checks whether the given coordinates fall within acceptable robot limits.
     * @param coordinate The coordinates to check
     * @return True if within range, false otherwise
     */
    private boolean withinRange(Pose2d coordinate) {
        boolean withinRange = !useLimits ||
                ((coordinate.position.x > MIN_X_DISTANCE &&
                        coordinate.position.x < PivotExtension.ExtensionState.INTAKE_EXTENSION.getValue() + 15)) &&
                        Math.abs(coordinate.position.y) < 7
//                        Math.abs(robot.pose.position.y + coordinate.position.y) < 15)
                        ;

        if (withinRange) {
            robot.tel.addData("withinRangeDist", coordinate.position.x);
        }
      /*  RobotLog.e("withinRange min_x: " + (coordinate.position.x > MIN_X_DISTANCE));
        RobotLog.e("withinRange ext cap: " + (coordinate.position.x < PivotExtension.ExtensionState.INTAKE_EXTENSION.getValue() + 15));
        RobotLog.e("withinRange y: " + (Math.abs(coordinate.position.y) < 10));
        RobotLog.e("withinRange pos: " + coordinate.position);
        RobotLog.e("withinRange: " + withinRange);

       */

        return withinRange;
    }

    public Action pathToTarget(boolean useRobotPose) {
        Pose2d pose = useRobotPose ? robot.pose : BasketPath.cyclePose;

        if (target.id == -1) {
            RobotLog.e("doesn't see sample");
            robot.tel.addLine("doesn't see sample");
            return new SequentialAction(); // Return an empty action if no target is detected
        }

        Pose2d currentPos = robot.pose;
        double heading = currentPos.heading.toDouble();

        // Calculate target position relative to the intake offsets
        Point relativeTarget = translatePoint(target.getRobotPos(), new Point(-INTAKE_X, -INTAKE_Y));
        // Convert robot-local target to field coordinates using current position and heading
        Point rotatedTarget = rotatePoint(relativeTarget, heading);
        Point fieldTarget = translatePoint(rotatedTarget, new Point(currentPos.position.x, currentPos.position.y));

        // Update extension target based on distance to target
        extensionLimelightTarget = Math.hypot(relativeTarget.x, relativeTarget.y);

        // Log positioning data
        robot.tel.addData("Robot Pos", currentPos.position.x + ", " + currentPos.position.y);
        robot.tel.addData("Field Target", fieldTarget.x + ", " + fieldTarget.y);
        robot.tel.addData("Robot Heading", Math.toDegrees(heading));
        RobotLog.e("Robot Pos: " + currentPos.position.x + ", " + currentPos.position.y);
        RobotLog.e("Field Target: " + fieldTarget.x + ", " + fieldTarget.y);
        RobotLog.e("Robot Heading: " + Math.toDegrees(heading));

        Point end;
        if (useRobotPose) {
            end = translatePoint(fieldTarget, rotatePoint(new Point(-8, 0), heading));
        } else {
            end = new Point(pose.position.x, fieldTarget.y);
        }
        RobotLog.e("robot heading: "+ pose.heading.toDouble());
        return robot.actionBuilder(currentPos)

                .strafeToLinearHeading(new Vector2d(end.x, end.y), pose.heading.toDouble())
                .build();
    }

    // Helper methods for point operations

    /**
     * Rotates a 2D point by the given angle (in radians).
     * @param p The point to rotate
     * @param a The angle in radians
     * @return The rotated point
     */
    private Point rotatePoint(Point p, double a) {
        double newX = p.x * Math.cos(a) + p.y * Math.sin(a);
        double newY = p.x * Math.sin(a) - p.y * Math.cos(a);
        return new Point(newX, newY);
    }

    /**
     * Translates a Pose2d by adding a translation vector.
     * @param p The pose to translate
     * @param t The translation vector
     * @return The translated point
     */
    private Point translatePoint(Pose2d p, Point t) {
        return new Point(p.position.x + t.x, p.position.y + t.y);
    }
    private Point translatePoint(Pose2d p, Pose2d t) {
        return new Point(p.position.x + t.position.x, p.position.y + t.position.y);
    }

    /**
     * Translates a point by adding two points.
     * @param p The original point
     * @param t The translation vector
     * @return The translated point
     */
    private Point translatePoint(Point p, Point t) {
        return new Point(p.x + t.x, p.y + t.y);
    }

    /**
     * Sets the current target sample.
     * @param s The sample to set as target
     */
    private void setTarget(Sample s) {
        this.target = s;
    }

    /**
     * Displays target information via telemetry.
     */
    public void displayTarget() {
        robot.tel.addData("Relative Bot Coordinate", "x: %.2f, y: %.2f", target.getRobotPos().position.x, target.getRobotPos().position.y);
        robot.tel.addData("Detected Class with limits", target.getType());
        robot.tel.addLine();

        LLResult latestResult = limelight.getLatestResult();
        if (latestResult != null) {
            List<LLResultTypes.DetectorResult> resultsList = latestResult.getDetectorResults();
            for (int i = 0; i < resultsList.size(); i++) {
                robot.tel.addData("Result " + (i + 1), resultsList.get(i).getClassName());
            }
        }

        robot.tel.addData("Confidence", "%.2f", target.getConfidence());
        robot.tel.addData("Priority", target.getMaxPriority());
    }

    /**
     * Action for searching and navigating to a sample.
     */
    public static class SearchForSample implements Action {
        private final ElapsedTime timer = new ElapsedTime();
        private final ElapsedTime timeInExtend = new ElapsedTime();
        private boolean firstWrite = false;
        private boolean secondWrite = false;
        private Action limelightPath;
        private int countColor = 0;
        private int countWrongColor = 0;
        private boolean retry = false;
        private final boolean useRobotPose;
        Action turn;
        double x;
        boolean sweep;

        int retries = 0;
        int cycle;
        public SearchForSample (boolean useRobotPose, boolean sweep, int cycle) {
            this.useRobotPose = useRobotPose;
            this.sweep = sweep;
            this.cycle = cycle;
            retries = 0;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (plotSamplesOnField) {
                for (Sample s : Limelight.allSamples) {
                    telemetryPacket.fieldOverlay()
                            .setStrokeWidth(1)
                            .setStroke(s.getType())
                            .strokeCircle(s.getFieldPos().position.x-5, s.getFieldPos().position.y, 2);
                }
            }
            if (!firstWrite) {
                Context.isColorSensor = true;
                firstWrite = true;
                timer.reset();
                robot.runAction(robot.actions.sweepAndExtend(sweep, retries));
                secondWrite = true;
                turn = robot.actionBuilder(robot.pose)
                        .turn(Math.toRadians(-10))
                        .turn(Math.toRadians(30))
//                        .strafeToConstantHeading(new Vector2d(robot.pose.position.x, robot.pose.position.y - 5))
                        .turn(Math.toRadians(-45))
//                        .strafeToConstantHeading(new Vector2d(robot.pose.position.x, robot.pose.position.y - 10))
                        .turn(Math.toRadians(45))
                        .turn(Math.toRadians(-30))
                        .turn(Math.toRadians(-10))
                        .turn(Math.toRadians(30))
                        .strafeToConstantHeading(new Vector2d(robot.pose.position.x, robot.pose.position.y - 5))
                        .turn(Math.toRadians(-45))
                        .strafeToConstantHeading(new Vector2d(robot.pose.position.x, robot.pose.position.y - 10))
                        .turn(Math.toRadians(45))
                        .turn(Math.toRadians(-30))
                        .turn(Math.toRadians(-10))
                        .turn(Math.toRadians(30))
                        .strafeToConstantHeading(new Vector2d(robot.pose.position.x, robot.pose.position.y + 5))
                        .turn(Math.toRadians(-45))
                        .strafeToConstantHeading(new Vector2d(robot.pose.position.x, robot.pose.position.y + 10))
                        .turn(Math.toRadians(45))
                        .turn(Math.toRadians(-30))
                        .build();

                countColor = 0;
                countWrongColor = 0;

                timeInExtend.reset();
            }

           /* if (isDoneSweeping && secondWrite) {
               // limelightPath = robot.limelight.pathToTarget(useRobotPose);
                //robot.runAction(robot.actions.limelightIntake(robot.limelight.extensionLimelightTarget));

                isDoneIntakeAction = false;
                retry = false;
                RobotLog.e("Target extension: " + robot.limelight.extensionLimelightTarget);
                secondWrite = false;
                isDoneSweeping = false;
            }

            */

            if (isDoneSweeping) {
                turn.run(telemetryPacket);
            }


            // Check for conditions to trigger a reset action
           /* if (!retry && (countWrongColor >= 5 || (countColor <= 0 && isDoneIntakeAction) || timer.milliseconds() > 5000)) {
                retry = true;
                timer.reset();
                Context.isColorSensor = false;
                robot.runAction(robot.actions.limelightReset(true));
                countWrongColor = 0;
                isDoneIntakeAction = false;
                robot.limelight.lookahead.remove(0);
                RobotLog.e("Resetting search for sample");
            }

            if (retry && timer.milliseconds() > 800) {
                firstWrite = false;
                retry = false;
                timer.reset();
                RobotLog.e("Reset complete, starting new action");
            }

            */

            // Execute the current limelight path action
//            if (firstWrite && !secondWrite) {
//
//            }

            // Monitor color sensor spikes for wrong (opposing) and correct (color/neutral) detections
            boolean wrongColorSpike = robot.intakeSpecimen.checkAgainstColors(
                    Context.color == Color.BLUE ? Color.RED : Color.BLUE);

            if (wrongColorSpike) {
                countWrongColor++;
            } else if (countWrongColor > 0) {
                countWrongColor--;
            }

            PivotExtension.ExtensionState.MANUAL.setValue(Math.min(PivotExtension.ExtensionState.INTAKE_EXTENSION.getValue(), PivotExtension.ExtensionState.START_EXTENSION_AUTO.getValue() + timer.seconds() * 3));
            boolean colorSpike = robot.intakeSpecimen.checkAgainstColors(Color.NEUTRAL, Context.color);
            if (colorSpike) {
                countColor++;
            } else if (countColor > 0) {
                countColor--;
            }
            RobotLog.e("CountColor:"+countColor);
            RobotLog.e("CountWrong:"+countWrongColor);

            robot.tel.addData("A - Count Color", countColor);


            if (countWrongColor > 5) {
                List<Task> extakeAndIntake = Builder.create()
                        .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_HOVER)
                        .delay(150)
                        .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.YEET)
                        .delay(200)
                        .moduleAction(robot.intakeSpecimen, IntakeSpecimen.WristState.SAMPLE_INTAKE)
                        .delay(200)
                        .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                        .moduleAction(robot.pivotExtension, PivotExtension.ExtensionState.MANUAL)
                        .executeCode(() -> PivotExtension.ExtensionState.MANUAL.setValue(Math.min(PivotExtension.ExtensionState.INTAKE_EXTENSION.getValue(), PivotExtension.ExtensionState.START_EXTENSION_AUTO.getValue() + retries * 3)))
                        .build();
                robot.runAction(extakeAndIntake);
                countWrongColor = 0;
                retries++;
            }

            if (robot.pivotExtension.extensionTarget == PivotExtension.ExtensionState.INTAKE_EXTENSION.getValue() && Math.abs(robot.pivotExtension.extensionPosition - robot.pivotExtension.extensionTarget) < 1) {
                 turn = robot.actionBuilder(robot.pose)
                         .turn(Math.toRadians(30))
                        .strafeToConstantHeading(new Vector2d(robot.pose.position.x, robot.pose.position.y - 5))
                        .turn(Math.toRadians(-45))
                        .strafeToConstantHeading(new Vector2d(robot.pose.position.x, robot.pose.position.y - 10))
                        .turn(Math.toRadians(45))
                        .turn(Math.toRadians(-30))
                         .build();
            }
            if (countColor > 5) {
                robot.runAction(robot.actions.highBasketExtensionCycle(false));
                Context.isColorSensor = false;
                countColor = 0;
                return false; // End action
            }
            return true;
        }
    }

    public static class LookaheadAction implements Action {
        private Action intakeCycle;
        private double ext;
        private final ElapsedTime timer = new ElapsedTime();
        private int countColor = 0;
        private int countWrongColor = 0;
        private boolean retry = false;
        private boolean firstWrite, secondWrite, doneWithPath;

        public LookaheadAction() {
            this.firstWrite = true;
            this.secondWrite = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstWrite) {
                Vector2d pos = robot.limelight.lookahead.get(0).fieldPos.position;

                double x = pos.x + PAST_DISTANCE - PivotExtension.ExtensionState.INTAKE_EXTENSION.getValue() - INTAKE_X;
                Pose2d endPose = new Pose2d(x, pos.y - INTAKE_Y, 0);

                ext = PivotExtension.ExtensionState.INTAKE_EXTENSION.getValue() - PRE_DISTANCE - PAST_DISTANCE;

                intakeCycle = basketCycle(0).endTrajectory().fresh()
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(endPose, Math.toRadians(0), AccelVelConstraint.VelConstraint(80), AccelVelConstraint.AccelConstraint(-40, 70))
                        .build();
                firstWrite = false;
                secondWrite = false;
            }

            if (robot.pivotExtension.pivotPosition < 30 && !secondWrite) {
                robot.pivotExtension.setState(PivotExtension.ExtensionState.MANUAL);
                PivotExtension.ExtensionState.MANUAL.setValue(ext);
                secondWrite = true;
            }



            if (!intakeCycle.run(telemetryPacket) && !doneWithPath) {
                robot.runAction(robot.actions.lookaheadIntake(ext));
                doneWithPath = true;
            }


            // Check for conditions to trigger a reset action
            /*if (!retry && (countWrongColor >= 5 || (countColor <= 0 && isDoneIntakeAction) || timer.milliseconds() > 5000)) {
                retry = true;
                timer.reset();
                Context.isColorSensor = false;
                robot.runAction(robot.actions.limelightReset(true));
                countWrongColor = 0;
                isDoneIntakeAction = false;
                robot.limelight.lookahead.remove(0);
                RobotLog.e("Resetting search for sample");
            }

            if (retry && timer.milliseconds() > 800) {
                retry = false;
                timer.reset();
                RobotLog.e("Reset complete, starting new action");
            }

            // Monitor color sensor spikes for wrong (opposing) and correct (color/neutral) detections
            boolean wrongColorSpike = robot.intakeSpecimen.checkAgainstColors(
                    Context.color == Color.BLUE ? Color.RED : Color.BLUE);

            if (wrongColorSpike) {
                countWrongColor++;
            } else if (countWrongColor > 0) {
                countWrongColor--;
            }

            boolean colorSpike = robot.intakeSpecimen.checkAgainstColors(Color.NEUTRAL, Context.color);
            if (colorSpike) {
                countColor++;
            } else if (countColor > 0) {
                countColor--;
            }
            RobotLog.e("Count:"+countColor);
            RobotLog.e("isColor:"+Context.isColorSensor);
            if (countColor > 15) {
                robot.runAction(robot.actions.highBasketExtensionCycle(false));
                Context.isColorSensor = false;
                return false; // End action
            }

             */
            return true;

        }
    }


    public static class LimelightSearchForSample implements Action {
        private final ElapsedTime timer = new ElapsedTime();
        private boolean firstWrite = false;
        private boolean secondWrite = false;
        private Action limelightPath;
        private int countColor = 0;
        private int countWrongColor = 0;
        private boolean retry = false;
        private boolean useRobotPose;
        public LimelightSearchForSample (boolean useRobotPose) {
            this.useRobotPose = useRobotPose;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!firstWrite) {
                firstWrite = true;
                timer.reset();
                isDoneSweeping = true;
                secondWrite = true;
                //robot.runAction(robot.actions.sweep());
                if (robot.limelight.target.id == -1){
                    return false;
                }
            }

            if (isDoneSweeping && secondWrite) {
                limelightPath = robot.limelight.pathToTarget(useRobotPose);
                robot.runAction(robot.actions.limelightDestroyBlockInTheWayIntake(robot.limelight.extensionLimelightTarget));
                isDoneIntakeAction = false;
                retry = false;
                RobotLog.e("Target extension: " + robot.limelight.extensionLimelightTarget);
                secondWrite = false;
                isDoneSweeping = false;
            }

            // Check for conditions to trigger a reset action
            if (!retry && (countWrongColor >= 5 || (countColor <= 0 && isDoneIntakeAction) || timer.milliseconds() > 4000)) {
                retry = true;
                timer.reset();
                Context.isColorSensor = false;
                robot.runAction(robot.actions.limelightReset(true));
                countWrongColor = 0;
                isDoneIntakeAction = false;
                RobotLog.e("Resetting search for sample");
            }

            if (retry && timer.milliseconds() > 800) {
                firstWrite = false;
                retry = false;
                timer.reset();
                RobotLog.e("Reset complete, starting new action");
            }

            // Execute the current limelight path action
            if (firstWrite && !secondWrite) {
                limelightPath.run(telemetryPacket);
            }

            // Monitor color sensor spikes for wrong (opposing) and correct (color/neutral) detections
//            boolean colorSpike = robot.intakeSpecimen.intake.isOverCurrent();
            boolean wrongColorSpike = robot.intakeSpecimen.checkAgainstColors(
                    Context.color == Color.BLUE ? Color.RED : Color.BLUE);

            if (wrongColorSpike) {
                countWrongColor++;
            } else if (countWrongColor > 0) {
                countWrongColor--;
            }

            boolean colorSpike = robot.intakeSpecimen.checkAgainstColors(Color.NEUTRAL, Context.color);
//            boolean colorSpike = robot.intakeSpecimen.checkAgainstColors(
//                    Context.color, Color.NEUTRAL);
            if (colorSpike) {
                countColor++;
            } else if (countColor > 0) {
                countColor--;
            }
            RobotLog.e("Count:"+countColor);
            if (countColor > 8) {
                robot.runAction(robot.actions.highBasketExtensionCycle(false));
                Context.isColorSensor = false;
                return false; // End action
            }
            return true;
        }
    }

    public static class searchForSampleRegionals implements Action {
        ElapsedTime timer = new ElapsedTime();
        boolean firstWrite = false;
        Action limelightPath;
        private boolean secondWrite = false;

        int countColor = 0, countWrongColor = 0;
        boolean retry = false, pathIsRunning = false;

        Vector2d holdPoint;

        public searchForSampleRegionals(Vector2d gotopoint) {
            holdPoint = gotopoint;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (!firstWrite) {
                firstWrite = true;
                timer.reset();
                limelightPath = robot.limelight.pathToTargetRegionals(holdPoint);
                robot.runAction(robot.actions.limelightDestroyBlockInTheWayIntake(robot.limelight.extensionLimelightTarget));
                isDoneIntakeAction = false;
                pathIsRunning = true;
                retry = false;
                isDoneSweeping = true;
                secondWrite = true;
                Context.isColorSensor = true;
                RobotLog.e("TARGETTTTT" + robot.limelight.extensionLimelightTarget);
            }

            if (!retry && (countWrongColor >= 5 || (countColor <= 0 && isDoneIntakeAction) || timer.milliseconds() > 3000)) {
                retry = true;
                timer.reset();
                robot.runAction(robot.actions.resetWithoutIntakeReset(true));
                countWrongColor = 0;
                isDoneIntakeAction = false;
                RobotLog.e("Resetting");
            }

            if (retry && timer.milliseconds() > 800) {
                firstWrite = false;
                retry = false;
                timer.reset();
                RobotLog.e("Done resetting, creating new action");
            }

          /*  if (pathIsRunning) {
                if (!limelightPath.run(telemetryPacket)) {
                    pathIsRunning = false;
                }
            } else {
                robot.hold(telemetryPacket);
            }

           */
            limelightPath.run(telemetryPacket);


            // Monitor color sensor spikes for wrong (opposing) and correct (color/neutral) detections
            boolean wrongColorSpike = robot.intakeSpecimen.checkAgainstColors(
                    Context.color == Color.BLUE ? Color.RED : Color.BLUE);

            if (wrongColorSpike) {
                countWrongColor++;
            } else if (countWrongColor > 0) {
                countWrongColor--;
            }

            boolean colorSpike = robot.intakeSpecimen.checkAgainstColors(Color.NEUTRAL, Context.color);
            if (colorSpike) {
                countColor++;
            } else if (countColor > 0) {
                countColor--;
            }
            RobotLog.e("CountColor:"+countColor);
            RobotLog.e("CountWrong:"+countWrongColor);

            robot.tel.addData("A - Count Color", countColor);


            if (countWrongColor > 5) {
                List<Task> extakeAndIntake = Builder.create()
                        .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.EXTAKE)
                        .delay(400)
                        .moduleAction(robot.intakeSpecimen, IntakeSpecimen.PowerState.INTAKE)
                        .build();
                robot.runAction(extakeAndIntake);
                countWrongColor = 0;

            }

            if (countColor > 5) {
                robot.runAction(robot.actions.highBasketExtensionCycle(false));
                Context.isColorSensor = false;
                countColor = 0;
                return false; // End action
            }
            return true;
        }
    }

    public Action pathToTargetRegionals(Vector2d hold) {
        if (target.id == -1) {
            RobotLog.e("doesn't see it");
            robot.tel.addLine("doesn't see sample");
            return new SequentialAction();
        }

        // Get current robot position (forward, sideways) and heading (radians)
        Pose2d pos = robot.pose;
        double heading = pos.heading.toDouble();
        // Creates point targetPos = the target relative to the INTAKE
        // target.getPos() is in (sideways, forward), relative to the ROBOT. that is why intake point is subtracted as (y,x)
        // translatePoint adds the two points' coordinates together
        Point targetPos = translatePoint(target.getFieldPos(), new Point(-INTAKE_X, -INTAKE_Y));
        //targetPos switched to (forward, sideways)
        // targetPos = new Point(targetPos.y, targetPos.x);
        // Calculate worldCoordinate = the targetPos (INTAKE coordinates) into FIELD coordinates
        // Rotate targetPos into field axis, then add it to the current robot position
        Point worldCoordinate = translatePoint(rotatePoint(targetPos, heading), new Point(pos.position.x, pos.position.y));
        // Calculate the targetHeading needed to turn the robot so the intake points at the sample.
        // Use targetPos.y but not targetPos.x because the rotation is done around the center of the robot, not around the intake
        // Both are .y because targetPos is switched to (forward, sideways), but target.getPos() is still in (sideways, forward)
        double targetHeading = heading + Math.atan2(targetPos.y, target.getRobotPos().position.x);
        //I just realized that we can get the drive points through intake_dist instead of the whole worldCoordinate thing
        double intakeDist = Math.sqrt(targetPos.x * targetPos.x + targetPos.y * targetPos.y);// + EXTENSION;
        //
        // Vector2d preWorldCoordinate = new Vector2d(
        // pos.position.x - (PRE_DISTANCE - intakeDist) * Math.cos(targetHeading),
        // pos.position.y - (PRE_DISTANCE - intakeDist) * Math.sin(targetHeading));
        // Vector2d pastWorldCoordinate = new Vector2d(
        // pos.position.x + (PAST_DISTANCE + intakeDist) * Math.cos(targetHeading),
        // pos.position.y + (PAST_DISTANCE + intakeDist) * Math.sin(targetHeading));
        extensionLimelightTarget = intakeDist;
        //
        // // Calculate the point before the block and the point after the block
        // // Add or Subtract the PRE/POST_DISTANCE, rotated into FIELD coordinates
        // /* To be honest, I don't know why intake_dist_from_center needs to exist, but when testing with chatgpt,
        // it looks like the robot is placed directly on top of the sample instead of being 12 inches before the sample. Weird.
        // */
        // Vector2d preWorldCoordinate = new Vector2d(
        // worldCoordinate.x - (PRE_DISTANCE) * Math.cos(targetHeading),
        // worldCoordinate.y - (PRE_DISTANCE) * Math.sin(targetHeading));
        // Vector2d pastWorldCoordinate = new Vector2d(
        // worldCoordinate.x + (PAST_DISTANCE) * Math.cos(targetHeading),
        // worldCoordinate.y + (PAST_DISTANCE) * Math.sin(targetHeading));

        robot.tel.addData("Robot Pos", pos.position.x + ", " + pos.position.y);
        robot.tel.addData("World Coordinate", worldCoordinate.x + ", " + worldCoordinate.y);
        robot.tel.addData("Robot Heading: ", Math.toDegrees(heading));
        robot.tel.addData("Target Heading: ", Math.toDegrees(targetHeading));

        // Turn first
        // Then drive to pre
        // Then drive to post
        // for now doesnt matter that they are distinct positions, but later we should make intake drop down at pre
        return robot.actionBuilder(pos)
                .strafeTo(hold)
                .turnTo(AngleUnit.normalizeRadians(targetHeading))
                .lineToX(hold.x + 0.1, AccelVelConstraint.VelConstraint(0.00000000000000000001), AccelVelConstraint.AccelConstraint(-0.00000000000000000001, 0.00000000000000000001))
                // .strafeToConstantHeading(preWorldCoordinate)
                // .strafeToConstantHeading(pastWorldCoordinate)
                .build();
    }
    public static class searchForWillis implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            System.out.println("i dont thinkhes ok like what was that like wait genuinely what just happened\n" +
                    "\n" +
                    "thats just called sleep talking right\n" +
                    "he does that??????\n" +
                    "everyone does when theyre rly tired\n" +
                    "\n" +
                    "no no not like that\n" +
                    "wait\n" +
                    "\n" +
                    "oh do u mean hes concerned for the bot or worried\n" +
                    "no i meant the weird hting he said or like\n" +
                    "cuz it kinda didnt make sense and cme out of nowhere\n" +
                    "\n" +
                    "he randmoly woke up and started talking abt how we needed to replace the\n" +
                    "abt ethna first righ\n" +
                    "oh yea abt how he was talking to ethan for 2 hours and and then how we needed to replace the box tubing standoffs or something\n" +
                    "\n" +
                    "wait was he dreaming\n" +
                    "\n" +
                    "i dont know but he didnt look ok like his eyes or smth did not look right\n" +
                    "\n" +
                    "um what\n" +
                    "\n" +
                    "i dont know\n" +
                    "\n" +
                    "should we just wake him up for real\n" +
                    "\n" +
                    "one time at like 2 am my lights were on my mom walked in and i was sleeping i started talking to her about task scheduler\n" +
                    "\n" +
                    "ok so hes fine hes just passionate about box tubing and ethan?!?!???!\n" +
                    "\n" +
                    "wait i think its lucky for him that it wasnt something else that was unrelated to robotics\n" +
                    "\n" +
                    "but what else would we bet hinking about like it has to be smth u were recently thinking abt to like idk subconciously think abt right\n" +
                    "\n" +
                    "true\n" +
                    "\n" +
                    "we need to make a " + new searchForWillis());
            return true;
        }
    }
}
