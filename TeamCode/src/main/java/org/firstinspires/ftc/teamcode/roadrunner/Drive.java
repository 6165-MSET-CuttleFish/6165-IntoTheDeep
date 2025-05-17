package org.firstinspires.ftc.teamcode.roadrunner;

import static org.firstinspires.ftc.teamcode.architecture.Context.readyForTapeMeasure;
import static org.firstinspires.ftc.teamcode.architecture.Robot.CONTROLLER;
import static org.firstinspires.ftc.teamcode.architecture.Robot.WRITE_TOGGLES;
import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.Robot;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.tele.DRIVE_TYPE;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * YOU WILL BE REPLACED
 * Drive has everything MecanumDrive had before, IT HAS EVERYTHING RR NEEDS EXCEPT PINPOINT
    * pinpoint is added in the subclass of Drive called "PinpointDrive"
 */
@Config
public class Drive {
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        public double inPerTick = 1; // If you're using OTOS/Pinpoint leave this at 1 (all values will be in inches, 1 tick = 1 inch)
        public double lateralInPerTick = 0.7000226990735313; // Tune this with LateralRampLogger (even if you use OTOS/Pinpoint)
        public double trackWidthTicks = 12.468384574910623;

        // feedforward parameters (in tick units)
        public double kS = 1.15;
        public double kV = 0.12;  /** WHERE DID OUR SIG FIGS GO RAHHHHH */
        public double kA = 0.035; /** WHERE DID OUR SIG FIGS GO RAHHHHH */

        // path profile parameters (in inches)
        /*public double maxWheelVel = 85;
        public double minProfileAccel = -80;
        public double maxProfileAccel = 100;
         */
        public double maxWheelVel = 60;
        public double minProfileAccel = -60;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = 4;//* 1.5; // shared with path
        public double maxAngAccel = 4;//* 1.5;

        // path controller gains
        public double axialGain = 16;
        public double lateralGain = 14;
        public double headingGain = 19; // shared with turn

        public double axialVelGain = 0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0; // shared with turn

        public double axialGainHold = 10.5;
        public double lateralGainHold = 9;
        public double headingGainHold = 10; // shared with turn

        public double axialVelGainHold = 1.2;
        public double lateralVelGainHold = 0.0;
        public double headingVelGainHold = 0.3; // shared with turn

        // end condition
        public double errorTolerance = 2;
        public double velocityTolerance = 0.5;
        public double timeout = 0.2; // in seconds, if other conditions are not met, it'll timeout after this one is met

    }

    public VoltageSensor voltageSensor;

    public static Params PARAMS = new Params();

    public TimeTrajectory t;
    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final EnhancedMotor fl, bl, br, fr;

    public LazyImu lazyImu;

    public final Localizer localizer;
    public Pose2d pose;
    public final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public double voltage;

    public Pose2d followerTarget = new Pose2d(0,0,0);


    public double time = System.nanoTime(), prevVelo = 0, count = 0, totalVelo = 0, totalTime = 0, accel = 0;
    public PoseVelocity2d velocity = new PoseVelocity2d(new Vector2d(0,0), 0);

    public boolean isBusy = true, canceled = false, hold = true;
    public Pose2dDual<Time> txWorldTarget;
    long time2 = System.nanoTime();
    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;
        private double lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;

        public DriveLocalizer() {
            leftFront = new OverflowEncoder(new RawEncoder(Drive.this.fl));
            leftBack = new OverflowEncoder(new RawEncoder(Drive.this.bl));
            rightBack = new OverflowEncoder(new RawEncoder(Drive.this.br));
            rightFront = new OverflowEncoder(new RawEncoder(Drive.this.fr));

            // TODO: reverse encoders if needed
            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        @Override
        public Twist2dDual<Time> update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();


            Rotation2d heading = Rotation2d.exp(0);

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }
    }

    public Drive(HardwareMap hardwareMap, Pose2d pose) {
        PARAMS = new Params();

        this.pose = pose;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        fl = new EnhancedMotor(hardwareMap, 0.1, "fl");
        bl = new EnhancedMotor(hardwareMap, 0.1,"bl");
        br = new EnhancedMotor(hardwareMap, 0.1, "br");
        fr = new EnhancedMotor(hardwareMap, 0.1,"fr");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        localizer = new DriveLocalizer();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }
        setDrivetrainPowers(
            wheelVels.leftFront.get(0) / maxPowerMag,
            wheelVels.leftBack.get(0) / maxPowerMag,
            wheelVels.rightBack.get(0) / maxPowerMag,
            wheelVels.rightFront.get(0) / maxPowerMag
        );


    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }

            followerTarget = new Pose2d(xPoints[xPoints.length - 1], yPoints[yPoints.length - 1], 0);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            time2 = System.nanoTime();
            RobotLog.e("SUS 0: " + getTime(time2));
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
                isBusy = true;
                if (robot != null) {
                    robot.t = timeTrajectory;
                }
            } else {
                t = Actions.now() - beginTs;
            }

            txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));
            velocity = updatePoseEstimate();
            Pose2d error = txWorldTarget.value().minusExp(pose);

            if (robot != null) {
                robot.runMarkers();
            }

          /* if ((t >= timeTrajectory.duration && error.position.norm() < PARAMS.errorTolerance && robotVelRobot.linearVel.norm() < PARAMS.velocityTolerance) || (t >= timeTrajectory.duration + PARAMS.timeout)) {
                setDrivetrainPowers(0,0,0,0);
                if (robot != null) {
                   robot.runMarkersForced();
                }
                return false;
            }

           */



            if (t >= timeTrajectory.duration || canceled) {

                RobotLog.e("t equals:  " +  t);
                canceled = false;
                isBusy = false;
                setDrivetrainPowers(0,0,0,0);

                if (robot != null) {
                    robot.runMarkersForced();
                    robot.t = null;
                }

                return false;
            }

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, velocity);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

            double fl = feedforward.compute(wheelVels.leftFront) / voltage; // / voltage;
            double bl = feedforward.compute(wheelVels.leftBack) / voltage; /// voltage;
            double br = feedforward.compute(wheelVels.rightBack) / voltage;  /// voltage;
            double fr = feedforward.compute(wheelVels.rightFront) / voltage; // / voltage;

            if (Context.type.equals(DRIVE_TYPE.MANUAL)) {
                setDrivetrainPowers(fl, bl, br, fr);
            }


            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));
            p.put("powers", fl + ", " + bl + ", " +  br + ", " +  fr);

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
            RobotLog.e("SUS 8: " + getTime(time2));


            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;
        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
                isBusy = true;
            } else {
                t = Actions.now() - beginTs;
            }

            robot.runMarkers();

            if (t >= turn.duration || canceled) {
                canceled = false;
                isBusy = false;
                setDrivetrainPowers(0,0,0,0);
                robot.runMarkersForced();
                return false;
            }

            txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            velocity = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, velocity);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

            double voltage;
            voltage = robot == null? voltageSensor.getVoltage() : robot.voltage;
            double fl = feedforward.compute(wheelVels.leftFront) / voltage;
            double bl = feedforward.compute(wheelVels.leftBack) / voltage;
            double br = feedforward.compute(wheelVels.rightBack) / voltage;
            double fr = feedforward.compute(wheelVels.rightFront) / voltage;

            setDrivetrainPowers(
                    fl, bl, br, fr
            );

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
    double getTime(long time) {
        return (System.nanoTime() - time) / Math.pow(10, 6);
    } // temporary but i have ocd

    public void setDrivetrainPowers(double fl, double bl, double br, double fr) {

        long time = System.nanoTime();

        if (WRITE_TOGGLES.drivetrainWrite && WRITE_TOGGLES.robotWrite) {
            this.fl.setPower(fl);
            this.bl.setPower(bl);
            this.br.setPower(br);
            this.fr.setPower(fr);
        }

        RobotLog.e("SUS -1: " + getTime(time));


    }


    public void hold(TelemetryPacket p) {

        time2 = System.nanoTime();

        if (!hold) return;


        RobotLog.e("SUS 0: " + getTime(time2));
        velocity = updatePoseEstimate();

        RobotLog.e("SUS 1: " + getTime(time2));
        PoseVelocity2dDual<Time> command = new HolonomicController(
                PARAMS.axialGainHold, PARAMS.lateralGainHold, PARAMS.headingGainHold,
                PARAMS.axialVelGainHold, PARAMS.lateralVelGainHold, PARAMS.headingVelGainHold
        )
                .compute(txWorldTarget, pose, velocity);

        RobotLog.e("SUS 2: " + getTime(time2));
        driveCommandWriter.write(new DriveCommandMessage(command));
        RobotLog.e("SUS 3: " + getTime(time2));

        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
        RobotLog.e("SUS 4: " + getTime(time2));
        final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
        RobotLog.e("SUS 5: " + getTime(time2));


        double fl = feedforward.compute(wheelVels.leftFront) / voltage; // / voltage;
        double bl = feedforward.compute(wheelVels.leftBack) / voltage; /// voltage;
        double br = feedforward.compute(wheelVels.rightBack) / voltage;  /// voltage;
        double fr = feedforward.compute(wheelVels.rightFront) / voltage; // / voltage;
        RobotLog.e("SUS 6: " + getTime(time2));

        setDrivetrainPowers(
                fl, bl, br, fr
        );
        RobotLog.e("SUS 7: " + getTime(time2));
        Canvas c = p.fieldOverlay();
        drawPoseHistory(c);

        c.setStroke("#4CAF50");
        Drawing.drawRobot(c, txWorldTarget.value());

        c.setStroke("#3F51B5");
        Drawing.drawRobot(c, pose);
        RobotLog.e("SUS 8: " + getTime(time2));

        isBusy = false;
    }

    public double getMagnitudeVelocity() {
        return Math.sqrt(Math.pow(velocity.linearVel.x, 2) + Math.pow(velocity.linearVel.y, 2));
    }

    public void getAccelMagnitude() {

        double newTime = System.nanoTime();
        double velo = getMagnitudeVelocity();

        totalVelo += velo - prevVelo;
        totalTime += (newTime - time) / (Math.pow(10, 9));

        if (count % 1 == 0) {
            accel = totalVelo / totalTime;
            totalTime = 0;
            totalVelo = 0;
            count = 1;
        } else {
            count++;
        }

        time = newTime;
        prevVelo = velo;

    }
}
