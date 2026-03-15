package org.firstinspires.ftc.teamcode.testops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Prism.Color;
import org.firstinspires.ftc.teamcode.hardware.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.hardware.Prism.PrismAnimations;

@TeleOp(name = "LED Test", group = "Test")
public class LedTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        waitForStart();

        while (opModeIsActive()) {
            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.Solid(Color.PINK));
        }
    }
}