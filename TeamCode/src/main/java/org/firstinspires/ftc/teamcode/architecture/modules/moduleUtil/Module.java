package org.firstinspires.ftc.teamcode.architecture.modules.moduleUtil;


import static org.firstinspires.ftc.teamcode.architecture.Robot.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.modules.IntakeSpecimen;

/**
 * Thanks Matthew for some comments, made my life a lot easier
 * See Marker to understand what abstract classes are and why it helps to have them
 * I was too lazy to add comments to individual modules but there's nothing in there covered that's not covered in here
    * Would suggest reading them to grasp how individual modules are implemented from this abstract superclass
 */
public abstract class Module {
    /**
     *  An array for the modules that have multiple states to them
     */
    public ModuleState[] states;
    public boolean constantUpdate;
    boolean stateChanged;
    boolean readUpdateChecker;
    boolean telemetryToggle = true;
    public String telIdentifier;
    public ElapsedTime timer;
    public enum Status {
        BUSY, IDLE
    }
    protected Status status = Status.IDLE;

    /**
     * constructor. gives module all information to do tasks. constantUpdate is used for modules with PID
     * or some other controller that needs constant updating
     */
    public Module(boolean constantUpdate) {
        this.constantUpdate = constantUpdate;
        timer = new ElapsedTime();
        initInternalStates();
        telIdentifier = this.getClass().toString().substring(this.getClass().toString().lastIndexOf('.') + 1);
        telemetryToggle = true;
    }
    public Module(boolean constantUpdate, boolean telemetryToggle) {
        this.constantUpdate = constantUpdate;
        timer = new ElapsedTime();
        initInternalStates();
        telIdentifier = this.getClass().toString().substring(this.getClass().toString().lastIndexOf('.') + 1);
        this.telemetryToggle = telemetryToggle;
    }


    /**
     * needed to store states of a module within this class (array to store if same class has multiple states)
     * ie: multiple servo deposit
     */
    protected void setInternalStates(ModuleState... states) {
        this.states=states;
    }


    /**
     * for modules w/ multiple states
     * iterate through all the states to find the ModuleState that matches
     */
    public void setState(ModuleState state) {
        for(int i = 0; i < states.length; i++) {
            if(state.getClass() == states[i].getClass()) {
                if(state != states[i]) {
                    states[i] = state;
                    onStateChange();
                }
            }
        }
    }

    /**
     * gets specific state you want
     */
    public ModuleState getState(Class e) {
        for (ModuleState state : states) {
            if (e == state.getClass()) {
                return state;
            }
        }
        return null;
    }

    public boolean within(ModuleState... c) {
        for (ModuleState p : c) {
            if (getState(p.getClass()).equals(p)) {
                return true;
            }
        }
        return false;
    }

    /**
     * triggers once state is changed
     */
    private void onStateChange() {
        timer.reset();
        readUpdateChecker = false;
        stateChanged = true;
        stateChanged();

        updateInternalStatus();
    }

    /**
     * way to call update loop from opMode
     * VERY IMPORTANT TO NOTE: constantUpdate affects read() too
        * So if you want servos to update only when state changes but also have color sensors which update every loop...
            * Yea that sucks...
        * If someone wants to change implementation to have both a constantReadUpdate and constantWriteUpdate, by all means
            * This is just slightly cleaner at the expense of functionality
     */
    public void readLoop() {
        if(constantUpdate || stateChanged) {
            read();
            readUpdateChecker=true;
        }
        updateInternalStatus();
        telemetryUpdate();
    }

    /**
     * way to call writes from opMode
     * There's no functional reason to separate reads and writes, it's just
     */
    public void writeLoop() {
        if(constantUpdate || stateChanged) {
            write();
            if(readUpdateChecker) {
                stateChanged=false;
            }
        }
    }

    public void init() {
        onStateChange();
    }

    /**
     * For all motor and servo writes (setPower() and setPosition())
     */
    protected abstract void write();

    /**
     * For all updating (encoder reads, sensor reads, etc)
     */
    protected abstract void read();

    /**
     * timer resets onStateChange()
     */
    public int timeSpentInState() {
        return (int) timer.milliseconds();
    }

    /**
     *  called once on stateChange (useful for things that only need to be called once when state changes)
     *  but also part of a module that is constantUpdate
     */
    protected void stateChanged(){}

    /**
     *  prints telemetry stuff. can rewrite implementation for it
     *  used for any information common to any module (e.g. timeSpentInState())
     */

    protected void telemetryUpdate() {
        if(telemetryToggle) {
            robot.tel.addLine("_________________");
            robot.tel.addData("Time Spent in State: ", timeSpentInState());
        }
    }

    /**
     *  need implementation to set the states so this module can access all states
     */
    protected abstract void initInternalStates();

    /**
     *  used to updating IDLE vs BUSY for each module
     */
    protected abstract void updateInternalStatus();
    public boolean isBusy() {
        return status == Status.BUSY;
    }
}
