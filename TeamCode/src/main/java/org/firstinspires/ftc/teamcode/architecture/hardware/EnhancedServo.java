package org.firstinspires.ftc.teamcode.architecture.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class EnhancedServo implements Servo {
    private final ServoImplEx servo;
    /**
     * caching tolerance refers to how much you're willing to let the position deviate from its current position before you setPosition()
     * you don't want to setPosition() more than you have to because every time you write to a servo, you destroy loop times
     */
    private double cachingTolerance;
    private double cachedPosition = Double.NaN;

    public EnhancedServo(Servo servo, double cachingTolerance) {
        this.servo = (ServoImplEx) servo;
        this.cachingTolerance = cachingTolerance;
    }

    public EnhancedServo(Servo servo) {
        this(servo, 0.01);
    }

    public EnhancedServo(HardwareMap hardwareMap, String name) {
        this(hardwareMap.get(Servo.class, name), 0.01);
    }

    public EnhancedServo(HardwareMap hardwareMap, double cachingTolerance, String name) {
        this(hardwareMap.get(Servo.class, name), cachingTolerance);
    }

    /**
     * the || statements just deal with edge cases
     * you want to turn the servo all the way to 0 if you set it to 0 regardless of if it deviates from current value or not
     * you want to turn the servo to 1/-1 if you set it to 1/-1 regardless of if it deviates from current value or not
     * @param position the new position of the servo, a value in the interval [-1.0, 1.0]
     */

    @Override
    public void setPosition(double position) {
        double corrected = Math.max(0.0, Math.min(1.0, position));
        if (Math.abs(corrected - cachedPosition) >= cachingTolerance
                || Double.isNaN(cachedPosition)) {
            cachedPosition = corrected;
            servo.setPosition(corrected);
        }
    }

    /**
     * difference between this and setPosition() is it returns whether it actually did the write or not
     */
    public boolean setPositionResult(double position) {
        double corrected = Math.max(0.0, Math.min(1.0, position));
        if (Math.abs(corrected - cachedPosition) >= cachingTolerance
                || Double.isNaN(cachedPosition)) {
            cachedPosition = corrected;
            servo.setPosition(corrected);
            return true;
        }
        return false;
    }

    public boolean setPositionRaw(double position) {
        double originalTolerance = this.cachingTolerance;
        this.cachingTolerance = 0.0;
        boolean result = setPositionResult(position);
        this.cachingTolerance = originalTolerance;
        return result;
    }

    // Delegate all other methods to the encapsulated Servo instance
    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
    }

    @Override
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }

    @Override
    public Manufacturer getManufacturer() {
        return servo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return servo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return servo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        servo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        servo.close();
    }

    @Override
    public double getPosition() {
        return servo.getPosition();
    }

    /** according to gm0, pwmDisable() turns some servos off but it keeps some holding at the position where it was disabled */
    public void pwm(boolean toggle) {
        if (toggle) {
            servo.setPwmEnable();
        } else {
            servo.setPwmDisable();
        }
    }




}
