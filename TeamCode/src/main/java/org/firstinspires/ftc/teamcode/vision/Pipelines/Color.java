package org.firstinspires.ftc.teamcode.vision.Pipelines;

/**
 * same Color class used for all uses of Color
    * Homography + AI
    * Intake Vision
    * Deciding which side we're playing on (which ties into the other two)
 * value maps to Limelight ID tagging for different colors
    * I think... vision crew help me out here if I'm wrong...
 */
public enum Color {
    RED(1), BLUE(0), NEUTRAL(2), NONE(1);

    public final int value;

    Color(int value) {
        this.value = value;
    }

}
