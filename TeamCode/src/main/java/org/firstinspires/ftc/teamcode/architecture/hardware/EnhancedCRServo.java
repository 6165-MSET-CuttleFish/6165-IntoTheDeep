package org.firstinspires.ftc.teamcode.architecture.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

public class EnhancedCRServo implements CRServo {
    private final CRServo crServo;

    /**
     * caching tolerance refers to how much you're willing to let the power deviate from its current power before you setPower()
        * you don't want to setPower() more than you have to because every time you write to a motor, you destroy loop times
     * See also @func
     */
    private double cachingTolerance;
    private double cachedPower = Double.NaN;

    /**
     * @param crServo the motor to encapsulate in the caching control
     * @param cachingTolerance the power delta threshold at which a motor write will occur.
     */
    public EnhancedCRServo(CRServo crServo, double cachingTolerance) {
        this.crServo = crServo;
        this.cachingTolerance = cachingTolerance;
    }

    public EnhancedCRServo(CRServo crServo) {
        this(crServo, 0.05);
    }

    public EnhancedCRServo(HardwareMap hardwareMap, String name) {
        this(hardwareMap.get(CRServo.class, name), 0.05);
    }

    public EnhancedCRServo(HardwareMap hardwareMap, double cachingTolerance, String name) {
        this(hardwareMap.get(CRServo.class, name), cachingTolerance);
    }

    /**
     * the || statements just deal with edge cases
        * you want to turn the motor all the way off if you set it to 0 regardless of if it deviates from current value or not
        * you want to turn the motor to max if you set it to 1/-1 regardless of if it deviates from current value or not
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     */
    @Override
    public void setPower(double power) {
        double corrected = Math.max(-1.0, Math.min(1.0, power));
        if (Math.abs(corrected - cachedPower) >= cachingTolerance
                || (corrected == 0.0 && cachedPower != 0.0)
                || (corrected >= 1.0 && !(cachedPower >= 1.0))
                || (corrected <= -1.0 && !(cachedPower <= -1.0))
                || Double.isNaN(cachedPower)) {
            cachedPower = corrected;
            crServo.setPower(corrected);
        }
    }

    /**
     * difference between this and setPower() is it returns whether it actually did the write or not
     */
    public boolean setPowerResult(double power) {
        double corrected = Math.max(-1.0, Math.min(1.0, power));
        if (Math.abs(corrected - cachedPower) >= cachingTolerance
                || (corrected == 0.0 && cachedPower != 0.0)
                || (corrected >= 1.0 && !(cachedPower >= 1.0))
                || (corrected <= -1.0 && !(cachedPower <= -1.0))
                || Double.isNaN(cachedPower)) {
            cachedPower = corrected;
            crServo.setPower(corrected);
            return true;
        }
        return false;
    }

    public boolean setPowerRaw(double power) {
        double originalTolerance = this.cachingTolerance;
        this.cachingTolerance = 0.0;
        boolean result = setPowerResult(power);
        this.cachingTolerance = originalTolerance;
        return result;
    }

    @Override
    public ServoController getController() {
        return crServo.getController();
    }

    @Override
    public int getPortNumber() {
        return crServo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        crServo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return crServo.getDirection();
    }

    @Override
    public double getPower() {
        return crServo.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return crServo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return crServo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return crServo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return crServo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        crServo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        crServo.close();
    }
}
