package org.firstinspires.ftc.teamcode.testops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Turret;

@TeleOp(name = "Turret Test", group = "Test")
public class TurretTestOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap);

        telemetry.addLine("Turret Test Ready");
        telemetry.addLine("Cross  -> go to 90 degrees");
        telemetry.addLine("Circle -> return to 0 degrees");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.cross) {
                turret.setAngle(90);
            } else if (gamepad1.circle) {
                turret.setAngle(0);
            }

            telemetry.addData("Target", turret.isAtTarget() ? "At target" : "Moving");
            telemetry.update();
        }
    }
}