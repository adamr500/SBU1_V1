package org.firstinspires.ftc.teamcode.testops;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

// Based on: https://www.ctrlaltftc.com/practical-examples/ftc-motor-control
@Config
@TeleOp(name = "Turret PID Test", group = "Test")
public class RPMPID extends LinearOpMode {

    static final double TICKS_PER_REV = 145.1;

    public static double Kp         = 0.001;
    public static double Ki         = 0.00000325;
    public static double Kd         = 0.0;
    public static double TARGET_RPM = 1000.0;
    public static double ALPHA = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "turret");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double integralSum = 0;
        double lastError   = 0;
        double error_filtered   = 0;
        ElapsedTime timer  = new ElapsedTime();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double targetTicksPerSec  = TARGET_RPM * TICKS_PER_REV / 60.0;
            double currentTicksPerSec = -(motor.getVelocity());
            double error              = targetTicksPerSec - currentTicksPerSec;
            error_filtered = ALPHA * error_filtered + (1- ALPHA)*error;

            double derivative = (error_filtered - lastError) / timer.seconds();
            integralSum      += error_filtered * timer.seconds();

            double power = (Kp * error_filtered) + (Ki * integralSum) + (Kd * derivative);
            power = Math.max(-1.0, Math.min(1.0, power));

            motor.setPower(power);

            lastError = error_filtered;
            timer.reset();

            double currentRPM = currentTicksPerSec / TICKS_PER_REV * 60.0;
            telemetry.addData("target  (RPM)", TARGET_RPM);
            telemetry.addData("current (RPM)", currentRPM);
            telemetry.addData("error   (RPM)", TARGET_RPM - currentRPM);
            telemetry.addData("power", power);
            telemetry.addData("ErrorFiltred", error_filtered);
            telemetry.update();
        }

        motor.setPower(0);
    }
}