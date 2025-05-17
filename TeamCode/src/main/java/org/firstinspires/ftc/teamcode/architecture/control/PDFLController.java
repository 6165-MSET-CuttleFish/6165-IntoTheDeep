package org.firstinspires.ftc.teamcode.architecture.control;

import com.acmerobotics.dashboard.config.Config;

/**
 * PDFL stands for Proportional, Derivative, Feedforward, Lower-Limit
 * The first two terms are FEEDBACK terms
    * It takes in information about the system to generate a response
        * In this case, this "information" is the error or change in error
 * The second two terms are FEEDFORWARD terms
    * FEEDFORWARD refers to an open loop guess one can make about the system
        * Therefore it does not take in any input from the plant (it doesn't read the error of the system)
        * We have feedforwards so the feedback loop can do less corrective work
            * In other words, we take advantage of the fact that WE KNOW THINGS ABOUT OUR SYSTEM!
            * A control system grounded in your knowledge of a system > corrective system that's empirically tuned
                * If you think about it, if you factored in everything about a system, it would have perfect control
                * Therefore, the closer you can get to that, the better your control is. That's why feedforward is god
    * Note that our Feedforward term (3rd term) is misnamed, it's only one type of Feedforward we can have
 * ____________________________________________________________________________________________________
 * Proportional = p * error
    * This makes the most logical sense, if you're further away from target, you want to add more power to get to the target
 * Derivative = d * Δerror/Δtime
    * If you think about the error vs time graph, the Δerror/Δtime is the SLOPE at any point
        * What the hell is a slope at a point?
            * Essentially, you're taking the slope between two points on the graph
                * But then you're bringing those points really really close to each other
            * In other words, it's the SLOPE OF THE TANGENT LINE at that point
        * If you're confused, search "What is a derivative" on google lol
 * Feedforward = f (a bunch of constants dealt with in one multiplier) * Power
    * Power refers to the Power to negate GRAVITY
        * We need an equal and opposite power to counteract this so our subsequent terms work symmetrically
    * Rather than calculating the power via torque calculations, we did a regression on two axes to REPRESENT THE POWER applied on the arm
    * Why does a 2 axis regression work:
        * Power proportional to Torque = R X F
            * R is the distance from the pivot
                * R != extension, it's not like we are moving the ENTIRE box tubing an R distance from the pivot
                * Hence, we made a regression to find R
                    * Well really it's R * C (constant multiplier) but all multipliers are offset by the f anyway
            * F is the force applied
                * F ≈ cos(θ) * C, but not REALLY because of inconsistencies in the system
                    * Therefore we made an angle regression to find F * C (another constant multiplier)
        * So now to find the torque, you just take R X F = Extension_Regression * Angle_Regression * C (resultant constant)
        * Why is power proportional to torque?
            * Power = Torque * angular velocity
                * So if it's not moving, it should take no power to hold something up??? RIGHT???
                    * No. Not all systems are perfect.
                * Just like when you hold something heavy up, it still requires power to hold it
                * Due to intrinsic motor characteristics (I didn't do enough research for this), motors are also inefficient in this sense
                    * Therefore STATIONARY TORQUE IS PROPORTIONAL TO CURRENT WHICH IS PROPORTIONAL TO POWER :)
 * Lower-Limit = l * direction (-1 or 1)
    * It just provides a constant power in the DIRECTION OF MOTION
        * in other words, TOWARDS THE TARGET
    * Meant to counteract KINETIC FRICTION, so even though the tuning process highlights a static friction coefficient, that's not the purpose of it
        * Because of this it's a mid solution at best, so honestly just changing the value until it works is the strat
    * If gravity compensation is not perfect it sometimes looks like this term is bugging out, just check gravity first
 * ____________________________________________________________________________________________________________
 * Tuning:
    * Find gravity compensation regressions FIRST
    * Tune for p, keep increasing until the system starts oscillating around the endpoint
    * Increase d until it starts jittering
    * If the system is converging BEFORE the endpoint, you can increase p
        * If there's no middle ground that works, add an l instead of increasing p to reach target instead
            * Better than doing l first because of the static friction vs kinetic friction issue mentioned above
    * If the system still overshoots with a maximized d, you must lower p
 **/

@Config
public class PDFLController {
    public double p = 0, d = 0, f = 0, l = 0;
    private double previousError;
    private double error;
    private double position;
    private double targetPosition;
    private double errorDerivative;
    private long previousUpdateTimeNano;
    private long deltaTimeNano;

    /**
     * This takes the current error and runs the PDFL on it.
     *
     * @return this returns the value of the PDFL from the current error.
     */
    public double getPDFL() {
        double feedback = error * p + errorDerivative * d;
        double feedforward = Math.signum(feedback) * l + f; /** probably wrong, it's really the signum of error but whatever */
        return feedback + feedforward;
    }

    /**
     * This can be used to update the PDFL's current position when inputting a current position and
     * a target position to calculate error. This will update the error from the current position to
     * the target position specified.
     * @param update This is the current position.
     */
    public void updatePosition(double update) {
        position = update;
        previousError = error;
        error = targetPosition - position;

        deltaTimeNano = System.nanoTime() - previousUpdateTimeNano;
        previousUpdateTimeNano = System.nanoTime();
        errorDerivative = (error - previousError) / (deltaTimeNano / Math.pow(10.0, 9));
    }

    public void update(double target, double current) {
        setTargetPosition(target);
        updatePosition(current);
    }
    public void reset() {
        previousError = 0;
        error = 0;
        position = 0;
        targetPosition = 0;
        errorDerivative = 0;
        previousUpdateTimeNano = System.nanoTime();
    }
    public void setTargetPosition(double set) {
        targetPosition = set;
    }
    public double getError() {
        return error;
    }

    public void setController(double p, double d, double f, double l) {
        this.p = p;
        this.d = d;
        this.f = f;
        this.l = l;
    }
}
