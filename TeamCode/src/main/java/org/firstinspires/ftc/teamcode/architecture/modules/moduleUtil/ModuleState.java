package org.firstinspires.ftc.teamcode.architecture.modules.moduleUtil;

public interface ModuleState {
    /**
     * Next season, HIGHLY RECOMMEND switching to objects from enums, there's just way more functionality
        * then this interface would become an abstract class
     * How I wish we didn't have to use interfaces
        * Unfortunately, with enums, Enum acts as the superclass, and you can't have more than one superclass
        * So this is gonna have to do
     * The problem with interfaces is that all values within them are considered static final
        * that means there's no way have each ModuleState have it's own "value" all shared through ModuleState
            * they all need their own values
        * However, we can make required functions (essentially abstract) to enforce all the states having values
     * If anyone has a better way to implement this, please do, it's really disgusting rn lol
     */

    /**
     * sets value you would set in a specific ModuleState
     */
    void setValue(double value);

    /**
     * gets value from a specific ModuleState
     */
    double getValue();
}