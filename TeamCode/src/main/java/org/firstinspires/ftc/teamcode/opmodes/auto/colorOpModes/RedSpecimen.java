package org.firstinspires.ftc.teamcode.opmodes.auto.colorOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.opmodes.auto.blueprints.Specimen5_1Auto;
import org.firstinspires.ftc.teamcode.vision.Pipelines.Color;

@Autonomous(name = "A - Red Specimen")
public class RedSpecimen extends Specimen5_1Auto {

    @Override
    protected void initialize() {
        Context.color = Color.RED;
        super.initialize();
    }
}
