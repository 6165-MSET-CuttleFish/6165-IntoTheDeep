package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.architecture.modules.Limelight;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Sample {
    ElapsedTime time = new ElapsedTime();
    public static double distanceWeight = 1;
    public static double angleWeight = 2;
    public static double proximityWeight = 125;
    public Pose2d robotPos;
    public Pose2d fieldPos;
    public int id;
    private double maxPriority = -Double.MAX_VALUE;
    private double confidence = -1;
    private Vector2d distanceToIntake;
    private Pose2d intakePos = new Pose2d(Limelight.INTAKE_X, Limelight.INTAKE_Y, 0);


    public Sample(Pose2d pos, int id) {
        this.robotPos = pos;
        this.id = id;
        this.distanceToIntake = distance(pos, intakePos);
    }

    public Sample(double x, double y, int id) {
        this.robotPos = new Pose2d(x, y, 0);
        this.id = id;
        this.distanceToIntake = distance(robotPos, intakePos);
    }

    public Sample(Pose2d pos, int id, double confidence) {
        this.robotPos = pos;
        this.id = id;
        this.confidence = confidence;
        this.distanceToIntake = distance(pos, intakePos);
    }

    public double getMaxPriority() {
        return maxPriority;
    }

    public Pose2d getRobotPos() {
        return this.robotPos;
    }
    public Pose2d getFieldPos() {
        return this.fieldPos;
    }

    public double getConfidence() {
        return this.confidence;
    }

    public String getType() {
        if (this.id == 0) {
            return "blue";
        }
        if (this.id == 1) {
            return "red";
        }
        if (this.id == 2) {
            return "yellow";
        }
        return "!!!Does not exist!!!";
    }

    private double getDistance(Sample sample) {
        return Math.sqrt(Math.pow(this.robotPos.position.x - sample.robotPos.position.x, 2) + Math.pow(this.robotPos.position.y - sample.robotPos.position.y, 2));
    }

    // radians angle between two samples
    private double getLateralDist(Sample sample) {
        // return Math.abs(Math.atan((this.pos.y-sample.pos.y)/(this.pos.x-sample.pos.x)));

        return Math.abs(robotPos.position.y - sample.robotPos.position.y);
    }

    //angle for a sample


    private double getPriorityFromDist() {
        return Math.abs(distanceWeight * (0.3 * distanceToIntake.x + (distanceToIntake.y + 2.7)));
    }

    // ease of access - larger distance better, larger angle better
    // so the smaller num this returns the easier to pick up
    public double checkProximity(Sample sample) {
        //double priority = (proximityWeight * getDistance(sample) /* angleWeight * getAngle(sample)*/)/(distanceWeight * distanceToIntake);

        double priority = getPriorityFromDist();

//        if (Math.abs(this.pos.position.x - sample.pos.position.x) > -0.1)  {
            if (Math.abs(this.robotPos.position.x) > Math.abs(sample.robotPos.position.x) && getLateralDist(sample) < 3.6) {
                priority += proximityWeight / (Math.pow((getDistance(sample) - 1), 2));


            }


        return priority;
    }

    public double checkAllPriority(ArrayList<Sample>... samples) {
        ArrayList<Sample> opposingSamples = new ArrayList<>();
        for (ArrayList<Sample> s : samples) {
            opposingSamples.addAll(s);
        }
        this.maxPriority = getPriorityFromDist();

        if (opposingSamples.size() <= 1) { // it includes itself
            return maxPriority;
        }


        for (Sample sample : opposingSamples) {
            double priority = checkProximity(sample);
            if (priority > this.maxPriority) {
                this.maxPriority = priority;
            }
        }

        return this.maxPriority;
    }

    public void calculateGlobalPos() {
        Pose2d currentPos = robot.pose;
        double heading = currentPos.heading.toDouble();

        // Convert robot-local target to field coordinates using current position and heading
        Point rotatedTarget = rotatePoint(getRobotPos(), heading);
        fieldPos = translatePoint(rotatedTarget, new Point(currentPos.position.x, currentPos.position.y));
    }

    /**
     * Rotates a 2D point by the given angle (in radians).
     * @param p The point to rotate
     * @param a The angle in radians
     * @return The rotated point
     */
    private Point rotatePoint(Pose2d p, double a) {
        double newX = p.position.x * Math.cos(a) + p.position.y * Math.sin(a);
        double newY = p.position.x * Math.sin(a) - p.position.y * Math.cos(a);
        return new Point(newX, newY);
    }

    /**
     * Translates a point by adding two points.
     * @param p The original point
     * @param t The translation vector
     * @return The translated point
     */
    private Pose2d translatePoint(Point p, Point t) {
        return new Pose2d(p.x + t.x, p.y + t.y, 0);
    }


    private Vector2d distance(Pose2d p1, Pose2d p2) {
        double dx = p2.position.x - p1.position.x;
        double dy = p2.position.y - p1.position.y;
        return new Vector2d(dx, dy);
    }
}