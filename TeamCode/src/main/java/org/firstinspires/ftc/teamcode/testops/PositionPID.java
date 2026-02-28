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
@TeleOp(name = "Position PID Test", group = "Test")
public class PositionPID extends LinearOpMode {

    public static double Kp             = 0.01;
    public static double Ki             = 0.001;
    public static double Kd             = 0.0001;
    public static int    TARGET_HIGH    = 5000;
    public static double TARGET_THRESHOLD = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "turret");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double integralSum = 0;
        double lastError   = 0;
        ElapsedTime timer  = new ElapsedTime();

        int target = 0;

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double current = -(motor.getCurrentPosition());
            double error   = target - current;

            double derivative = (error - lastError) / timer.seconds();
            integralSum      += error * timer.seconds();

            double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            power = Math.max(-1.0, Math.min(1.0, power));

            motor.setPower(power);

            lastError = error;
            timer.reset();

            if (Math.abs(error) < TARGET_THRESHOLD) {
                target = (target == 0) ? TARGET_HIGH : 0;
                integralSum = 0;
            }

            telemetry.addData("setpoint (ticks)", target);
            telemetry.addData("current  (ticks)", (int) current);
            telemetry.addData("error    (ticks)", (int) error);
            telemetry.addData("power",             power);
            telemetry.update();
        }

        motor.setPower(0);
    }
}