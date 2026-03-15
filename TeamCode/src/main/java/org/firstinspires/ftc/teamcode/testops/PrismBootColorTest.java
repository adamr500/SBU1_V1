package org.firstinspires.ftc.teamcode.testops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Prism.Color;
import org.firstinspires.ftc.teamcode.hardware.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.hardware.Prism.PrismAnimations;

//@TeleOp(name = "Prism Boot Color Test", group = "Test")
public class PrismBootColorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        telemetry.addLine("Ready — press Start to set boot colour to pink snake");
        telemetry.update();

        waitForStart();

        // Adjust this to change how fast the snakes move (0.0 = stopped, 1.0 = fastest)
        float snakeSpeed = 0.2f;

        // Build a pink snakes animation
        PrismAnimations.Snakes pinkSnake = new PrismAnimations.Snakes(Color.PINK);
        pinkSnake.setSpeed(snakeSpeed);

        // Clear any existing animations then push the pink snake onto layer 0
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, pinkSnake);

        // Save the current animation state into artboard 0
        prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);

        // Set artboard 0 as the default boot artboard and enable it
        prism.setDefaultBootArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        prism.enableDefaultBootArtboard(true);

        telemetry.addLine("Done! Pink snake set as default boot colour.");
        telemetry.addData("Firmware", prism.getFirmwareVersionString());
        telemetry.update();

        // Hold until OpMode is stopped so telemetry stays visible
        while (opModeIsActive()) {
            idle();
        }
    }
}