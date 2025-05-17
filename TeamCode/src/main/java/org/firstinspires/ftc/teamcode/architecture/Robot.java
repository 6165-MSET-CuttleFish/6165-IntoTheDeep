package org.firstinspires.ftc.teamcode.architecture;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.markers.Marker;
import org.firstinspires.ftc.teamcode.architecture.modules.Hang;
import org.firstinspires.ftc.teamcode.architecture.modules.IntakeSpecimen;
import org.firstinspires.ftc.teamcode.architecture.modules.Limelight;
import org.firstinspires.ftc.teamcode.architecture.modules.PivotExtension;
import org.firstinspires.ftc.teamcode.architecture.modules.moduleUtil.Module;
import org.firstinspires.ftc.teamcode.architecture.modules.moduleUtil.ModuleState;
import org.firstinspires.ftc.teamcode.architecture.opmodes.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Task;
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.TaskScheduler;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.vision.Sample;

import java.util.ArrayList;
import java.util.List;

/** oh my jesus christ the lord there's a lot of stuff here */
@Config
public class Robot extends PinpointDrive {

    /** keep our static robot in Robot rather than in Context... why not? It's more compact and self-contained
     * VERY IMPORTANT: THIS ROBOT DOESN'T UPDATE WHEN YOU MAKE A NEW ROBOT BECAUSE IT IS STATIC
        * However, EVERYTHING ELSE inside of robot does because they AREN'T STATIC
        * If that's confusing, just treat it like it's not in the Robot class at all
            * The functionality is the same regardless of which class you put it in, whether Context or Robot or whatever else
     */
    public static Robot robot;

    /** opMode is initialized in robot
     * this value is passed to coroutines to let the coroutine know that the opMode is being run
        * yes, I know I said we won't use LinearOpMode anymore, only EnhancedOpMode
            * However, since EnhancedOpMode extends LinearOpMode, it deals with that case ALONG WITH any LinearOpMode cases
            * Just in case someone wants to initialize robot NOT in EnhancedOpMode for whatever reason...
            * If yall wanna switch opMode to be of type EnhancedOpMode, be my guest, but there's no reason to
     */
    public EnhancedOpMode opMode;

    /** RobotActions is in robot now, NOT STATIC
     * therefore you now call robot.actions (object) rather than Context.actions (static)
     */
    public RobotActions actions;

    /**
     * same as with actions, telemetry is now in robot, NOT STATIC
        * therefore you call robot.tel.addData(...) instead of Context.tel
     */
    public final MultipleTelemetry tel;
    public final TaskScheduler scheduler;

    public IntakeSpecimen intakeSpecimen;
    public PivotExtension pivotExtension;
    public Limelight limelight;
    public Hang hang;

    /** Array to store looptimes for avg **/
    public double[] looptimes = new double[10];
    private int looptimeIndex = 0;

    /** stores list of generic markers
     * it calls run() whenever current path is active (see method runMarkers() below)
     * when path finishes, it forces all the markers that didn't execute to run (see method runMarkersForced() below)
     */
    public ArrayList<Marker> markerList = new ArrayList<>();

    /**
     * even if all the other writes are true, if robotWrite is false, none of the writes will work
     * drivetrainWrite only works if all writes go through setDrivetrainPowers(). PLS USE IT LOL thanks
     */
    /** FTCLib Gamepads... you don't have to use this for joysticks if you don't want to, they're very convenient for buttons/triggers */
    private final GamepadEx g1, g2;

    /**
     * Button Readers Functions
     * button.wasJustPressed()
     * this returns true if the button was NOT PRESSED prior and then became PRESSED
     * Note: this is not when it's released, it's when it's first touched
     * button.wasJustReleased()
     * if was PRESSED prior and then became NOT PRESSED
     * button.isDown()... pretty self-explanatory
     * YOU SHOULDN'T NEED ANY OTHER BUTTON FUNCTIONS FOR ANY FUNCTIONALITY
     * these 3 functions in tandem solve all combinations (see ButtonReader class for why that is)
     * See Trigger Reader Functions in TriggerReader class I'm too lazy
     */
    public KeyReader[] keyReaders;
    public static class CONTROLLER {
        public ButtonReader A1, B1, Y1, X1,
                LSB1, RSB1,
                DPAD_UP1, DPAD_DOWN1, DPAD_LEFT1, DPAD_RIGHT1,
                LB1, RB1,
                A2, B2, Y2, X2,
                LSB2, RSB2,
                DPAD_UP2, DPAD_DOWN2, DPAD_LEFT2, DPAD_RIGHT2,
                LB2, RB2;
        public TriggerReader
                LT1, RT1, LT2, RT2;
    }
    public static class WRITE_TOGGLES {
        public boolean hangWrite = true;
        public boolean pivotExtensionWrite = true;
        public boolean intakeSpecimenWrite = true;
        public boolean limelightWrite = true;
        public boolean drivetrainWrite = true;
        public boolean robotWrite = true;
    }
    public static CONTROLLER CONTROLLER = new CONTROLLER();
    public static WRITE_TOGGLES WRITE_TOGGLES = new WRITE_TOGGLES();

    public Robot(EnhancedOpMode opMode, Pose2d pose) throws InterruptedException {

        super(opMode.hardwareMap, pose);

        this.opMode = opMode;
        this.actions = new RobotActions();
        this.tel = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.scheduler = new TaskScheduler();
        this.pivotExtension = new PivotExtension(opMode.hardwareMap);
        this.intakeSpecimen = new IntakeSpecimen(opMode.hardwareMap);
        this.limelight = new Limelight(opMode.hardwareMap);
        this.hang = new Hang(opMode.hardwareMap);
        this.g1 = new GamepadEx(opMode.gamepad1);
        this.g2 = new GamepadEx(opMode.gamepad2);

        initializeKeyReaders();
        init();

        IntakeSpecimen.cameraActive = false; // override in opMode
    }

    public void read() {
        pivotExtension.readLoop();
        intakeSpecimen.readLoop();
        hang.readLoop();
        limelight.readLoop();

        voltage = voltageSensor.getVoltage();

        looptimeIndex += 1;
        looptimes[looptimeIndex % 10] = opMode.time;
    }

    /**
     * Drivetrain writes DO NOT HAPPEN HERE
        * see Drive.setDrivetrainPowers() and command click for all usages
            * I'm too lazy to put put it in an array and send it here, but if someone wants to, be my guest
     *
     */
    public void write() {
        if (WRITE_TOGGLES.robotWrite) {
            if (WRITE_TOGGLES.pivotExtensionWrite) {
                pivotExtension.writeLoop();
            }
            if (WRITE_TOGGLES.intakeSpecimenWrite) {
                intakeSpecimen.writeLoop();
            }
            if (WRITE_TOGGLES.hangWrite) {
                hang.writeLoop();
            }
            if (WRITE_TOGGLES.limelightWrite) {
                limelight.writeLoop();
            }
        }
    }


    public void init() {
        pivotExtension.init();
        intakeSpecimen.init();
        hang.init();
        limelight.init();
    }
    public void runAction(List<Task> tasks) {
        scheduler.scheduleTaskList(tasks);
    }
    public void runIntakeSpecimen(ModuleState m) { setModule(intakeSpecimen, m);}
    public void runPivotExtension(ModuleState m) { setModule(pivotExtension, m);}
    public void runHang(ModuleState m) { setModule(hang, m);}
    private void setModule(Module m, ModuleState state) {
        for (ModuleState c: m.states) {
            if (state.getClass() == c.getClass()) {
                m.setState(state);
                return;
            }
        }
        throw new NullPointerException("State is not in Module!");

    }

    /**
     * This does two things:
        * It initializes the KeyReaders (which gamepad, which button)
        * It stores a reference to the readers in an array
            * this array is used to update them all in EnhancedOpMode through a for loop
     */
    public void initializeKeyReaders() {
         keyReaders = new KeyReader[]{
                CONTROLLER.A1 = new ToggleButtonReader(g1, GamepadKeys.Button.A),
                CONTROLLER.B1 = new ToggleButtonReader(g1, GamepadKeys.Button.B),
                CONTROLLER.Y1 = new ToggleButtonReader(g1, GamepadKeys.Button.Y),
                CONTROLLER.X1 = new ToggleButtonReader(g1, GamepadKeys.Button.X),
                CONTROLLER.LSB1 = new ToggleButtonReader(g1, GamepadKeys.Button.LEFT_STICK_BUTTON),
                CONTROLLER.RSB1 = new ToggleButtonReader(g1, GamepadKeys.Button.RIGHT_STICK_BUTTON),
                CONTROLLER.DPAD_UP1 = new ToggleButtonReader(g1, GamepadKeys.Button.DPAD_UP),
                CONTROLLER.DPAD_DOWN1 = new ToggleButtonReader(g1, GamepadKeys.Button.DPAD_DOWN),
                CONTROLLER.DPAD_LEFT1 = new ToggleButtonReader(g1, GamepadKeys.Button.DPAD_LEFT),
                CONTROLLER.DPAD_RIGHT1 = new ToggleButtonReader(g1, GamepadKeys.Button.DPAD_RIGHT),
                CONTROLLER.LB1 = new ToggleButtonReader(g1, GamepadKeys.Button.LEFT_BUMPER),
                CONTROLLER.RB1 = new ToggleButtonReader(g1, GamepadKeys.Button.RIGHT_BUMPER),
                CONTROLLER.A2 = new ToggleButtonReader(g2, GamepadKeys.Button.A),
                CONTROLLER.B2 = new ToggleButtonReader(g2, GamepadKeys.Button.B),
                CONTROLLER.Y2 = new ToggleButtonReader(g2, GamepadKeys.Button.Y),
                CONTROLLER.X2 = new ToggleButtonReader(g2, GamepadKeys.Button.X),
                CONTROLLER.LSB2 = new ToggleButtonReader(g2, GamepadKeys.Button.LEFT_STICK_BUTTON),
                CONTROLLER.RSB2 = new ToggleButtonReader(g2, GamepadKeys.Button.RIGHT_STICK_BUTTON),
                CONTROLLER.DPAD_UP2 = new ToggleButtonReader(g2, GamepadKeys.Button.DPAD_UP),
                CONTROLLER.DPAD_DOWN2 = new ToggleButtonReader(g2, GamepadKeys.Button.DPAD_DOWN),
                CONTROLLER.DPAD_LEFT2 = new ToggleButtonReader(g2, GamepadKeys.Button.DPAD_LEFT),
                CONTROLLER.DPAD_RIGHT2 = new ToggleButtonReader(g2, GamepadKeys.Button.DPAD_RIGHT),
                CONTROLLER.LB2 = new ToggleButtonReader(g2, GamepadKeys.Button.LEFT_BUMPER),
                CONTROLLER.RB2 = new ToggleButtonReader(g2, GamepadKeys.Button.RIGHT_BUMPER),
                CONTROLLER.LT1 = new TriggerReader(g1, GamepadKeys.Trigger.LEFT_TRIGGER),
                CONTROLLER.RT1 = new TriggerReader(g1, GamepadKeys.Trigger.RIGHT_TRIGGER),
                CONTROLLER.LT2 = new TriggerReader(g2, GamepadKeys.Trigger.LEFT_TRIGGER),
                CONTROLLER.RT2 = new TriggerReader(g2, GamepadKeys.Trigger.RIGHT_TRIGGER)
        };
    }
    public Pose2d samplePos(Pose2d robotPose, Sample sample) {
        double c = Math.sqrt(Math.pow(sample.robotPos.position.x, 2) + Math.pow(sample.robotPos.position.y, 2));
        robot.tel.addData("c", c);
        double heading = 0;
        if (sample.robotPos.position.y != 0){
            heading = Math.atan(sample.robotPos.position.x/sample.robotPos.position.y);
        }
        robot.tel.addData("heading", heading);
        return new Pose2d(
                robotPose.position.x + c * Math.sin(heading + robotPose.heading.toDouble()),
                robotPose.position.y + c * Math.cos(heading + robotPose.heading.toDouble()),
                0
        );
    }
    public void runMarkers() {
        for (Marker m: markerList) {
            if (!m.isDone) {
                m.isDone = !m.run();
            }
        }
    }
    public void runMarkersForced() {
        for (Marker m: markerList) {
            if (!m.isDone) {
                m.action.run();
            }
        }
        markerList.clear();
    }

    public double getCurrent() {
        double c = 0.0;
        for (LynxModule module : opMode.hardwareMap.getAll(LynxModule.class)) {
            c += module.getCurrent(CurrentUnit.AMPS);
        }
        return c;
    }

    public double getAvgLooptime() {
        double t = 0;
        for (double l : looptimes) { t += l; }
        return t / 10;
    }
    public double getServoBusCurrent(LynxModule bus) {
        return bus.getGpioBusCurrent(CurrentUnit.AMPS);
    }
}
