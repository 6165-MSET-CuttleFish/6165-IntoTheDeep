package org.firstinspires.ftc.teamcode.opmodes.auto.colorOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.opmodes.auto.blueprints.CycleAuto;
import org.firstinspires.ftc.teamcode.vision.Pipelines.Color;

@Disabled
@Autonomous(name = "B - Blue Cycle")
public class BlueCycle extends CycleAuto {

    @Override
    protected void initialize() {
        Context.color = Color.BLUE;
        super.initialize();
    }
}
