package org.firstinspires.ftc.teamcode.opmodes.auto.blueprints;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.pathing.Specimen5_1Path;
import org.firstinspires.ftc.teamcode.pathing.Specimen5_2Path;

public class Specimen5_2Auto extends Auto {
    @Override
    protected void autoInit() {
        Specimen5_2Path specimenPath = new Specimen5_2Path(Context.POSES.specimen5_1Start);
        action = specimenPath.createPath();
        action.preview(c);
    }

    @Override
    protected boolean shouldUpdatePoseEstimate() {
        return true;
    }
}