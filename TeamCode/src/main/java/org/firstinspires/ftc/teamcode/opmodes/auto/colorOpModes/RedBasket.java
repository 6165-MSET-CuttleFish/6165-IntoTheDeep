package org.firstinspires.ftc.teamcode.opmodes.auto.colorOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.Context;
import org.firstinspires.ftc.teamcode.opmodes.auto.blueprints.BasketAuto;
import org.firstinspires.ftc.teamcode.vision.Pipelines.Color;

@Autonomous(name = "A - Red Basket", preselectTeleOp = "A - Tele")
public class RedBasket extends BasketAuto {

    @Override
    protected void initialize() {
        Context.color = Color.RED;
        super.initialize();
    }
    @Override
    public void initializeLoop() {
        super.initializeLoop();
    }
    @Override
    public void onStart() {
        super.onStart();
    }

    @Override
    protected void primaryLoop() {
        super.primaryLoop();
    }
}
