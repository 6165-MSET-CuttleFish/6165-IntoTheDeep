package org.firstinspires.ftc.teamcode.architecture.modules;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;
import org.firstinspires.ftc.teamcode.architecture.modules.moduleUtil.Module;
import org.firstinspires.ftc.teamcode.architecture.modules.moduleUtil.ModuleState;
import org.firstinspires.ftc.teamcode.architecture.task_scheduler.Builder;
import org.firstinspires.ftc.teamcode.vision.Pipelines.Color;

@Config
public class IntakeSpecimen extends Module {
    // Configuration constants
    public static boolean telemetryToggle = false;
    public static boolean showWrist = false;
    public static boolean showClaw = false;
    public static boolean showSweeper = false;
    public static boolean showFlipper = false;
    public static boolean showIntakePower = false;
    public static boolean showIntakeVelocity = false;
    public static boolean showColorSensors = true;
    public static boolean printCurrent = false;
    public static boolean isRawPos = false;
    public static boolean cameraActive = false;

    // Manual position controls
    public static double rawWristPosition = 0;
    public static double rawFlipperPosition = 0;

    // Hardware components
    public final EnhancedMotor intake;
    private final EnhancedServo wrist;
    private final EnhancedServo claw;
    public final EnhancedServo headlight;
    private final EnhancedServo leftSweeper;
    private final EnhancedServo rightSweeper;
    private final EnhancedServo flipper;
    private final AnalogInput wristAnalog;
    public RevColorSensorV3 colorSensorLeft;
    public RevColorSensorV3 colorSensorRight;

    // State variables
    public double intakePower;
    public double wristPosition;
    public double clawPosition;
    public double leftSweeperPosition;
    public double rightSweeperPosition;
    public double flipperPosition;
    public double intakeVelocity, intakePosition;
    public double lightValue;
    public boolean firstWrite = true;
    public boolean isSetIntakeState = true;
    public double manualWristOffset = 0;

    // Color sensing variables
    public Color activeColor = Color.NONE;
    public int countColor = 0;
    private float red, green, blue, alpha;
    private double total;
    private double redInd, blueInd, greenInd;
    private double leftDist, rightDist;

    // Current sensing
    boolean isStalled = false;
    private int countStall = 0;

    /**
     * Flipper position states
     */
    public enum FlipperState implements ModuleState {
        DOWN(0.59),
        UP(0.65);

        private double value;

        FlipperState(double value) {
            this.value = value;
        }

        @Override
        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = value;
        }
    }

    public enum LightState implements ModuleState {
        ON(1),
        OFF(0);

        private double value;

        LightState(double value) {
            this.value = value;
        }

        @Override
        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = value;
        }
    }

    /**
     * Left sweeper position states
     */
    public enum LeftSweeperState implements ModuleState {
        IN(0.51),
        OUT(0.17);

        private double value;

        LeftSweeperState(double value) {
            this.value = value;
        }

        @Override
        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = value;
        }
    }

    /**
     * Right sweeper position states
     */
    public enum RightSweeperState implements ModuleState {
        IN(0.8),
        OUT(0.5);

        private double value;

        RightSweeperState(double value) {
            this.value = value;
        }

        @Override
        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = value;
        }
    }

    /**
     * Wrist position states
     */
    public enum WristState implements ModuleState {
        INIT(0.41),
        SPECIMEN_PICKUP(0.94),
        SPECIMEN_DEPOSIT(0.64),
        SPECIMEN_FRONT_DEPOSIT(0.81),
        SPECIMEN_OLD_DEPOSIT(0.82),
        SAMPLE_HOVER(0.63),
        SAMPLE_READY(0.63),
        SAMPLE_INTAKE(0.72),
        SAMPLE_ADJUST_INTAKE(0.75),
        SAMPLE_TOP_INTAKE(0.84),
        CLAW_DROPDOWN(0.08),
        SAMPLE_ACTIVE_DEPOSIT(0.08),
        SAMPLE_CLAW_DEPOSIT(0.39),
        SAMPLE_ANGLED_DEPOSIT(0.14),
        SAMPLE_ACTIVE_LOW_DEPOSIT(0.14),
        HANG_SETUP(0.31),
        HANG_HANGING_1(0.37),
        RESET_WRIST(0.915); // when we reset the servo spline

        private double value;

        WristState(double value) {
            this.value = value;
        }

        @Override
        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = value;
        }
    }

    /**
     * Claw position states
     */
    public enum ClawState implements ModuleState {
        LOOSE_CLAW(.47),
        CLOSED(0.37),
        OPEN(0.75);

        private double value;

        ClawState(double value) {
            this.value = value;
        }

        @Override
        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = value;
        }
    }

    /**
     * Intake power states
     */
    public enum PowerState implements ModuleState {
        INTAKE(-1),
        EXTAKE(0.8),

        SLOW_EXTAKE(200),
        SCORE(800),
        SCORE_AUTO(1500),
        HOLDING(-1),
        OFF(0),
        SLOW_SCORE(300),
        YEET_THIRD(0.4),
        YEET(0.5),
        YEETMORE(1),
        ADA(0.4),
        DOUBLE_EXTAKE(10);

        private double value;

        PowerState(double value) {
            this.value = value;
        }

        @Override
        public double getValue() {
            return value;
        }

        @Override
        public void setValue(double value) {
            this.value = value;
        }
    }

    /**
     * Constructor for the IntakeSpecimen module
     * @param hardwareMap Hardware map from the opMode
     */
    public IntakeSpecimen(HardwareMap hardwareMap) {
        super(true, telemetryToggle);

        // Initialize hardware components
        intake = new EnhancedMotor(hardwareMap, "intake");
        headlight = new EnhancedServo(hardwareMap, 0.1, "headlight");
        wrist = new EnhancedServo(hardwareMap, "wrist");
        claw = new EnhancedServo(hardwareMap, "claw");
        leftSweeper = new EnhancedServo(hardwareMap, "leftSweeper");
        rightSweeper = new EnhancedServo(hardwareMap, "rightSweeper");
        flipper = new EnhancedServo(hardwareMap, "flipper");
        wristAnalog = hardwareMap.get(AnalogInput.class, "wristAnalog");

        // Initialize color sensors
        colorSensorLeft = hardwareMap.get(RevColorSensorV3.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(RevColorSensorV3.class, "colorSensorRight");

        // Configure intake motor
        intake.setCurrentAlert(3.5, CurrentUnit.AMPS);
        intake.setMode(STOP_AND_RESET_ENCODER);
        intake.setMode(RUN_WITHOUT_ENCODER);
        intake.setTargetPositionTolerance(3);
    }

    @Override
    protected void write() {
        // Set sweeper positions
        leftSweeper.setPosition(leftSweeperPosition);
        rightSweeper.setPosition(rightSweeperPosition);

        if (!robot.pivotExtension.isReset) {
            // Handle wrist and flipper positioning
            if (isRawPos) {
                wrist.setPosition(rawWristPosition);
                flipper.setPosition(rawFlipperPosition);
            } else {
                wrist.setPosition(wristPosition + manualWristOffset);
                if (robot.pivotExtension.pivotTarget < 90) {
                    flipperPosition = FlipperState.DOWN.getValue();
                }
                flipper.setPosition(flipperPosition);
            }

            // Handle intake motor control
            PowerState currentPowerState = (PowerState) getState(PowerState.class);
            if (currentPowerState.equals(PowerState.SCORE_AUTO) ||
                    currentPowerState.equals(PowerState.SCORE) ||
                    currentPowerState.equals(PowerState.SLOW_SCORE) ||
                    currentPowerState.equals(PowerState.SLOW_EXTAKE)
            ) {

                if (!intake.getMode().equals(RUN_USING_ENCODER)) {
                    intake.setMode(RUN_USING_ENCODER);
                }
                intake.setVelocity(currentPowerState.getValue());
            } else if (currentPowerState.equals(PowerState.DOUBLE_EXTAKE)) {
                if (firstWrite) {
                    firstWrite = false;
                    intake.setPower(PowerState.SLOW_EXTAKE.getValue());
                    intake.setTargetPosition((int) (intake.getCurrentPosition() + PowerState.DOUBLE_EXTAKE.getValue()));
                    robot.runAction(
                            Builder.create()
                                    .executeCode(() -> isSetIntakeState = false)
                                    .await(() -> Math.abs(intake.getCurrentPosition() - intake.getTargetPosition()) < 5)
                                    .moduleAction(robot.intakeSpecimen, PowerState.HOLDING)
                                    .executeCode(() -> isSetIntakeState = true)
                                    .executeCode(() -> firstWrite = true)
                                    .build()
                    );
                }
                if (!intake.getMode().equals(RUN_TO_POSITION)) {
                    intake.setMode(RUN_TO_POSITION);
                }
            } else {
                if (!intake.getMode().equals(RUN_WITHOUT_ENCODER)) {
                    intake.setMode(RUN_WITHOUT_ENCODER);
                }
                intake.setPower(intakePower);
            }

            // Set claw position
            claw.setPosition(clawPosition);
        } else if (robot.pivotExtension.letWristMove) {
            setState(WristState.SAMPLE_INTAKE);
            wrist.setPosition(wristPosition);
        }

        if (intakePower != 0) {
            countStall += isStalled ? (countStall < 10 ? 1 : 0) : (countStall > 0 ? -1 : 0);
        }

        headlight.setPosition(lightValue);
    }

    @Override
    protected void read() {
        // Read current state values
        wristPosition = getState(WristState.class).getValue();
        clawPosition = getState(ClawState.class).getValue();
        if (isSetIntakeState) {
            intakePower = getState(PowerState.class).getValue();
        }
        leftSweeperPosition = getState(LeftSweeperState.class).getValue();
        rightSweeperPosition = getState(RightSweeperState.class).getValue();
        flipperPosition = getState(FlipperState.class).getValue();
        intakeVelocity = intake.getVelocity();
        intakePosition = intake.getCurrentPosition();
        isStalled = intake.isOverCurrent();
        lightValue = getState(LightState.class).getValue();

        // Update color sensor data if enabled
        if (Context.isColorSensor) {
             updateColors();
        }
    }

    @Override
    public void telemetryUpdate() {
        if (telemetryToggle) {
            robot.tel.addData("specimen", robot.getServoBusCurrent(
                    robot.opMode.hardwareMap.get(LynxModule.class, "Expansion Hub 2")));

            super.telemetryUpdate();

            if (showClaw) {
                robot.tel.addData("Intake claw state", getState(ClawState.class));
                robot.tel.addData("clawPosition", claw.getPosition());
            }

            if (showIntakePower) {
                robot.tel.addData("Intake power state", getState(PowerState.class));
                robot.tel.addData("intakePower", intake.getPower());
            }

            if (showIntakeVelocity) {
                robot.tel.addData("Intake velocity", intakeVelocity);
                robot.tel.addData("wrist pos", wristAnalog.getVoltage() / 3.3 * 360);
                robot.tel.addData("Intake pos ", intakePosition);
            }

            if (showWrist) {
                if (isRawPos) {
                    robot.tel.addData("rawWristPos", rawWristPosition);
                } else {
                    robot.tel.addData("Intake position state", getState(WristState.class));
                    robot.tel.addData("wristPosition", wrist.getPosition());
                    robot.tel.addData("manualWristOffset", manualWristOffset);
                }
            }

            if (showSweeper) {
                if (!isRawPos) {
                    robot.tel.addData("Sweeper position state", getState(WristState.class));
                }
                robot.tel.addData("leftSweeperPosition", leftSweeper.getPosition());
                robot.tel.addData("rightSweeperPosition", rightSweeper.getPosition());
            }

            if (showFlipper) {
                if (isRawPos) {
                    robot.tel.addData("rawFlipperPos", rawFlipperPosition);
                } else {
                    robot.tel.addData("Flipper position state", getState(WristState.class));
                    robot.tel.addData("flipperPosition", flipperPosition);
                }
            }

            if (showColorSensors) {
                robot.tel.addData("red", red);
                robot.tel.addData("green", green);
                robot.tel.addData("blue", blue);
                robot.tel.addData("alpha", alpha);
                robot.tel.addData("total", total);
                robot.tel.addData("indRed", redInd);
                robot.tel.addData("indBlue", blueInd);
                robot.tel.addData("indNeutral", greenInd);
                robot.tel.addData("distLeft", leftDist);
                robot.tel.addData("distRight", rightDist);
                robot.tel.addData("COLOR", activeColor);
            }

            if (printCurrent) {
                robot.tel.addData("intakeCurrent", getCurrent());
            }
        }
    }

    /**
     * Get the current consumption of the intake motor
     * @return Current in amps
     */
    public double getCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    protected void initInternalStates() {
        setInternalStates(
                WristState.INIT,
                ClawState.CLOSED,
                PowerState.OFF,
                LeftSweeperState.IN,
                RightSweeperState.IN,
                FlipperState.DOWN,
                LightState.OFF
        );
    }

    @Override
    protected void updateInternalStatus() {
        status = Status.IDLE;
    }

    public int getCountStall() {
        return countStall;
    }

    /**
     * Updates color sensor readings and determines the active color
     */
    public void updateColors() {
        NormalizedRGBA left = colorSensorLeft.getNormalizedColors();
        NormalizedRGBA right = colorSensorRight.getNormalizedColors();

        float redLeft = left.red;
        float greenLeft = left.green;
        float blueLeft = left.blue;
        float alphaLeft = left.alpha;
        float redRight = right.red;
        float greenRight = right.green;
        float blueRight = right.blue;
        float alphaRight = right.alpha;

        red = Math.max(redLeft, redRight);
        green = Math.max(greenLeft, greenRight);
        blue = Math.max(blueLeft, blueRight);
        alpha = Math.max(alphaRight, alphaLeft);

        total = red + green + blue + alpha + 0.0001;
        double redRatio = (double) red / total;
        double greenRatio = (double) green / total;
        double blueRatio = (double) blue / total;

        redInd = redRatio * 1000;
        blueInd = blueRatio * 1000;
        greenInd = greenRatio * 1000;

        activeColor = determineColor(redInd, blueInd, greenInd);


        leftDist = colorSensorLeft.getDistance(DistanceUnit.INCH);
        rightDist = colorSensorRight.getDistance(DistanceUnit.INCH);



        countColor += checkAgainstColors(Color.NEUTRAL, Context.color) ? 1 : -1;
        countColor = Math.max(0, countColor);
    }


    /**
     * Checks if the active color matches any of the specified colors
     * @param colors Array of colors to check against
     * @return True if there's a match, false otherwise
     */
    public boolean checkAgainstColors(Color... colors) {
        for (Color c : colors) {
            if (activeColor == c) {
                return true;
            }
        }
        return false;
    }

    /**
     * Determines the color based on color sensor readings and distances
     * @param redInd Red component indicator
     * @param blueInd Blue component indicator
     * @param greenInd Green component indicator
     * @return Determined color
     */
    private Color determineColor(double redInd, double blueInd, double greenInd) {
        if ((leftDist > 1.7) && (rightDist > 1.7)) {
            return Color.NONE;
        }

        if (redInd > blueInd && redInd > greenInd) {
            return Color.RED;
        }

        if (blueInd > redInd) {
            return Color.BLUE;
        }

        return Color.NEUTRAL;
    }
}
