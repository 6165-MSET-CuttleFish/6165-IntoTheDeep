package org.firstinspires.ftc.teamcode.architecture.markers;

/**
 * Markers are a great place to learn what abstract methods and classes are
 * ABSTRACT ESSENTIALLY SIGNIFIES THE CLASS BEING A BLUEPRINT
    * That means anything that EXTENDS an abstract class MUST have ALL the abstract functions
        * In this case, run() is the blueprint that all subclasses must follow
        * that means all the markers (Instant, Temporal, Spatial) MUST have a run() function
        * If you make a class extend Marker without a run(), it will throw an error
    * Why is this a helpful tool?
        * Instead of making an array of instant markers, temporal markers, and spatial markers...
        * You just have an array of Markers, and you're allowed to call run() on all of them because THEY ALL HAVE THE METHOD
        * Helps make code cleaner, this is the same solution used in EnhancedOpMode and Modules
 */
public abstract class Marker {

    /**
     * isDone is used so that the marker isn't run more than once
     */
    public boolean isDone = false;

    /**
     * action is whatever the marker's job is
        * A Runnable can be any LAMBDA call, it's just a piece of code to execute
        * LAMBDAS are just executables to run whenever Runnable.run() is executed
        * To make a lambda, just say Runnable r = () -> roboticsKid.getALife()
            * then if r.run() is called, it calls roboticsKid.getALife() in that moment
     * action can be accessed in all subclasses of Marker, so it doesn't have to be declared anywhere else
        * hence why if you command-click to "action" in SpatialMarker or TemporalMarker, it'll redirect here
     */
    public Runnable action;

    /**
     * true should keep the action running, false should turn the action off, see Robot for implementation of that
     */
    public abstract boolean run();
}
