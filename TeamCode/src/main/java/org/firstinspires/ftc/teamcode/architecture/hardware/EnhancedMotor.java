package org.firstinspires.ftc.teamcode.architecture.hardware;

import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class EnhancedMotor implements DcMotorEx {
    private final DcMotorEx dcMotorEx;
    /**
     * caching tolerance refers to how much you're willing to let the power deviate from its current power before you setPower()
     * you don't want to setPower() more than you have to because every time you write to a motor, you destroy loop times
     */
    private double cachingTolerance;
    private double cachedPower = Double.NaN;
    private boolean voltageCompensation;
    public static double dampen = 0;

    /**
     * @param dcMotorEx the motor to encapsulate in the caching control
     * @param cachingTolerance the power delta threshold at which a motor write will occur.
     */
    public EnhancedMotor(DcMotorEx dcMotorEx, double cachingTolerance) {
        this.dcMotorEx = dcMotorEx;
        this.cachingTolerance = cachingTolerance;
    }

    public EnhancedMotor(DcMotorEx dcMotorEx) {
        this(dcMotorEx, 0.05);
    }
    public EnhancedMotor(HardwareMap hardwareMap, String name) {
        this(hardwareMap.get(DcMotorEx.class, name), 0.05);
    }
    public EnhancedMotor(HardwareMap hardwareMap, double cachingTolerance, String name) {
        this(hardwareMap.get(DcMotorEx.class, name), cachingTolerance);
    }


    /**
     * the || statements just deal with edge cases
     * you want to turn the motor all the way off if you set it to 0 regardless of if it deviates from current value or not
     * you want to turn the motor to max if you set it to 1/-1 regardless of if it deviates from current value or not
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     */
    @Override
    public void setPower(double power) {
//        double compensation = (dampen + (12.0 / robot.voltage)) / (dampen + 1);
        double scaleFactor = 1.5;
        double compensation = Math.pow((10 / robot.voltage), scaleFactor);
        double corrected = power * compensation;

        corrected = Math.max(-1.0, Math.min(1.0, corrected)); // Coerce power to [-1.0, 1.0]


        if (Math.abs(corrected - cachedPower) >= cachingTolerance
                || (corrected == 0.0 && cachedPower != 0.0)
                || (corrected >= 1.0 && !(cachedPower >= 1.0))
                || (corrected <= -1.0 && !(cachedPower <= -1.0))
                || Double.isNaN(cachedPower)) {
            cachedPower = corrected;
            dcMotorEx.setPower(corrected);
        }
    }

    /**
     * difference between this and setPower() is it returns whether it actually did the write or not
     */
    public boolean setPowerResult(double power) {
        power = power / robot.voltage * 12.0;
        double corrected = Math.max(-1.0, Math.min(1.0, power)); // Coerce power to [-1.0, 1.0]

        if (Math.abs(corrected - cachedPower) >= cachingTolerance
                || (corrected == 0.0 && cachedPower != 0.0)
                || (corrected >= 1.0 && !(cachedPower >= 1.0))
                || (corrected <= -1.0 && !(cachedPower <= -1.0))
                || Double.isNaN(cachedPower)) {
            cachedPower = corrected;
            dcMotorEx.setPower(corrected);
            return true;
        }
        return false;
    }

    /**
     * Sets cachingTolerance to 0 temporarily, then performs and returns setPowerResult
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     * @return if a hardware write to update the output to the motor was executed
     */
    public boolean setPowerRaw(double power) {
        double originalTolerance = this.cachingTolerance;
        this.cachingTolerance = 0.0;
        boolean result = setPowerResult(power);
        this.cachingTolerance = originalTolerance;
        return result;
    }

    // Delegate all other methods to the encapsulated DcMotorEx instance
    @Override
    public void setMotorEnable() {
        dcMotorEx.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        dcMotorEx.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return dcMotorEx.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        dcMotorEx.setVelocity(angularRate);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        dcMotorEx.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return dcMotorEx.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return dcMotorEx.getVelocity(unit);
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        dcMotorEx.setPIDCoefficients(mode, pidCoefficients);
    }
    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) {
        dcMotorEx.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {

    }

    @Override
    public void setPositionPIDFCoefficients(double p) {

    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return null;
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return dcMotorEx.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        dcMotorEx.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return dcMotorEx.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return dcMotorEx.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
       return dcMotorEx.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        dcMotorEx.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
       return dcMotorEx.isOverCurrent();
    }

    @Override
    public void setMode(RunMode mode) {
        dcMotorEx.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return dcMotorEx.getMode();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return dcMotorEx.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        dcMotorEx.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return dcMotorEx.getController();
    }

    @Override
    public int getPortNumber() {
        return dcMotorEx.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        dcMotorEx.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return dcMotorEx.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        dcMotorEx.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return dcMotorEx.getPowerFloat();
    }

    @Override
    public boolean isBusy() {
        return dcMotorEx.isBusy();
    }

    @Override
    public void setTargetPosition(int position) {
        dcMotorEx.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return dcMotorEx.getTargetPosition();
    }

    @Override
    public int getCurrentPosition() {
        return dcMotorEx.getCurrentPosition();
    }

    @Override
    public void setDirection(Direction direction) {
        dcMotorEx.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return dcMotorEx.getDirection();
    }

    @Override
    public double getPower() {
        return dcMotorEx.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return dcMotorEx.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return dcMotorEx.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return dcMotorEx.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return dcMotorEx.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        dcMotorEx.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        dcMotorEx.close();
    }
}