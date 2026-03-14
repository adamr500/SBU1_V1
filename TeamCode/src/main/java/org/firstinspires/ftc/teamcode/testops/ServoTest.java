package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
//@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends LinearOpMode {

    public static double HoodPosition = 0.5;
    public static double StopperPosition = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo hoodRight = hardwareMap.get(Servo.class, "hoodRight");
        Servo stopper = hardwareMap.get(Servo.class, "stopper");
        Servo hoodLeft = hardwareMap.get(Servo.class, "hoodLeft");

        waitForStart();

        while (opModeIsActive()) {
            hoodRight.setPosition(HoodPosition);
            stopper.setPosition(StopperPosition);
            hoodLeft.setPosition(1-HoodPosition);

            telemetry.addData("Hood Right position", HoodPosition);
            telemetry.addData("Stopper", StopperPosition);
            telemetry.addData("Hood Left position", 1-HoodPosition);
            telemetry.update();
        }
    }
}