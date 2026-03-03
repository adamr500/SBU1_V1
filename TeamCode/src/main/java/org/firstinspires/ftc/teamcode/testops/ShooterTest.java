package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {

    static final double TICKS_PER_REV = 28.0;

    public static double Kp         = 0.001;
    public static double Ki         = 0.00000325;
    public static double Kd         = 0.0;
    public static double TARGET_RPM = 3000.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx shooterLeft  = hardwareMap.get(DcMotorEx.class, "ShooterLeft");
        DcMotorEx shooterRight = hardwareMap.get(DcMotorEx.class, "ShooterRight");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);

        double integralSum = 0;
        double lastError   = 0;
        ElapsedTime timer  = new ElapsedTime();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double targetTicksPerSec  = TARGET_RPM * TICKS_PER_REV / 60.0;
            double currentTicksPerSec = shooterLeft.getVelocity();
            double error              = targetTicksPerSec - currentTicksPerSec;

            double derivative = (error - lastError) / timer.seconds();
            integralSum      += error * timer.seconds();

            double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            power = Math.max(-1.0, Math.min(1.0, power));

            shooterLeft.setPower(power);
            shooterRight.setPower(power);

            lastError = error;
            timer.reset();

            double currentRPM = currentTicksPerSec / TICKS_PER_REV * 60.0;
            telemetry.addData("target  (RPM)", TARGET_RPM);
            telemetry.addData("current (RPM)", currentRPM);
            telemetry.addData("error   (RPM)", TARGET_RPM - currentRPM);
            telemetry.addData("power", power);
            telemetry.update();
        }

        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }
}