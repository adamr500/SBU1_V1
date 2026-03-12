package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Odo Pod Test", group = "Test")
public class OdoPodTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            telemetry.addData("Status",   pinpoint.getDeviceStatus());
            telemetry.addData("X (in)",   pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y (in)",   pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}