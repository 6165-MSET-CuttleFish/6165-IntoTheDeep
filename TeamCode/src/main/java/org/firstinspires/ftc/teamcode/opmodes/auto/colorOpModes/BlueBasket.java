package org.firstinspires.ftc.teamcode.opmodes.auto.colorOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.opmodes.auto.blueprints.BasketAuto;
import org.firstinspires.ftc.teamcode.vision.Pipelines.Color;

@Autonomous(name = "A - Blue Basket", preselectTeleOp = "A - Tele")
public class BlueBasket extends BasketAuto {

    @Override
    protected void initialize() {
        Context.color = Color.BLUE;
        super.initialize();
    }
}
