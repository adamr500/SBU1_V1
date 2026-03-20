package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@Config
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {

    static final double TICKS_PER_REV = 28.0;

    public static double shooterPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx shooterLeft  = hardwareMap.get(DcMotorEx.class, "ShooterLeft");
        DcMotorEx shooterRight = hardwareMap.get(DcMotorEx.class, "ShooterRight");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            shooterLeft.setPower(shooterPower);
            shooterRight.setPower(shooterPower);

            double leftVel  = Math.abs(shooterLeft.getVelocity());
            double rightVel = Math.abs(shooterRight.getVelocity());
            double avgTicksPerSec = (leftVel + rightVel) / 2.0;
            double currentRPM = avgTicksPerSec / TICKS_PER_REV * 60.0;

            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Left  Velocity (ticks/s)", leftVel);
            telemetry.addData("Right Velocity (ticks/s)", rightVel);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.update();
        }

        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }
}