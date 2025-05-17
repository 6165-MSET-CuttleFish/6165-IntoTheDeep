package org.firstinspires.ftc.teamcode.opmodes.auto.blueprints;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.pathing.SpecimenPath;

public class SpecimenAuto extends Auto {
    @Override
    protected void autoInit() {
        SpecimenPath specimenPath = new SpecimenPath(Context.POSES.specimenStart);
        action = specimenPath.createPath();
        action.preview(c);
    }

    @Override
    protected boolean shouldUpdatePoseEstimate() {
        return true;
    }
}