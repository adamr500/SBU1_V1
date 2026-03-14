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
@TeleOp(name = "Position PID", group = "Test")
public class PositionPID extends LinearOpMode {

    public static double Kp               = 0.00001;
    public static double Ki               = 0;
    public static double Kd               = 0;
    public static int    TARGET_POSITION  = 500;  // set this on dashboard before pressing R1

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "Turret");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double integralSum = 0;
        double lastError   = 0;
        ElapsedTime timer  = new ElapsedTime();

        int target = 0;
        boolean r1Held = false;

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            // Hold R1 → go to TARGET_POSITION; release → return to 0
            boolean r1Now = gamepad1.right_bumper;
            if (r1Now && !r1Held) {
                target = TARGET_POSITION;
                integralSum = 0;
            } else if (!r1Now && r1Held) {
                target = 0;
                integralSum = 0;
            }
            r1Held = r1Now;

            double current = motor.getCurrentPosition();
            double error   = target - current;

            double dt = timer.seconds();
            timer.reset();

            double derivative = (dt > 0) ? (error - lastError) / dt : 0;
            integralSum      += error * dt;

            double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            power = Math.max(-1.0, Math.min(1.0, power));

            motor.setPower(power);
            lastError = error;

            telemetry.addData("target  (ticks)", target);
            telemetry.addData("current (ticks)", (int) current);
            telemetry.addData("error   (ticks)", (int) error);
            telemetry.addData("power",            power);
            telemetry.addData("hold R1 = go to TARGET_POSITION, release = return to 0", "");
            telemetry.update();
        }

        motor.setPower(0);
    }
}