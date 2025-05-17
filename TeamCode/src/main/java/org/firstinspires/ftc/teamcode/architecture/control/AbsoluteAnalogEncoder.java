package org.firstinspires.ftc.teamcode.architecture.control;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;

@Config
public class AbsoluteAnalogEncoder implements HardwareDevice {
    public static double DEFAULT_RANGE = 1;
    private final AnalogInput encoder;
    private double offset, analogRange;
    private boolean inverted;
    private boolean wraparound;

    public AbsoluteAnalogEncoder(AnalogInput enc){
        this(enc, DEFAULT_RANGE);
    }
    public AbsoluteAnalogEncoder(AnalogInput enc, double aRange){
        encoder = enc;
        analogRange = aRange;
        offset = 0;
        inverted = false;
    }

    public AbsoluteAnalogEncoder zero(double off){
        offset = off;
        return this;
    }
    public AbsoluteAnalogEncoder setInverted(boolean invert){
        inverted = invert;
        return this;
    }

    public void setWraparound(boolean wraparound) {
        this.wraparound = wraparound;
    }

    public boolean getDirection() {
        return inverted;
    }

    /**
     * the voltage by default does a wraparound (obviously voltage doesn't scale infinitely)
     * therefore to make the encoder not wraparound (e.g. for extension)
        * you just add one rotation when the voltage wraps around as it does on line 50
     **/
    public double getCurrentPosition() {
        double pos = (!inverted ? 1 - getVoltage() / analogRange : getVoltage() / analogRange) * Math.PI*2 - offset;
        if (wraparound && pos > Math.PI * 1.5) {
            pos -= Math.PI * 2;
        }

        pos *= 180 / Math.PI;

        if (pos < 0) {
            pos += 360;
        }
        return pos;
    }


    public AnalogInput getEncoder() {
        return encoder;
    }

    /**
     * for some reason, getVoltage() returns a value from 0-1 rather than 0-3.3
        * This is adjusted for by changing the DEFAULT_RANGE from 3.3 to 1
        * Note: DEFAULT_RANGE is the value of analogRange (set in constructor)
     */
    public double getVoltage(){
        return encoder.getVoltage();
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }
    @Override
    public String getDeviceName() {
        return "AbsoluteAnalogEncoder";
    }
    @Override
    public String getConnectionInfo() {
        return null;
    }
    @Override
    public int getVersion() {
        return 0;
    }
    @Override
    public void resetDeviceConfigurationForOpMode() {

    }
    @Override
    public void close() {

    }
}