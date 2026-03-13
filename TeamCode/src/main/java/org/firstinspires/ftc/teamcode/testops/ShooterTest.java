package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {

    static final double TICKS_PER_REV = 28.0;

    public static double Kp         = 0.01;
    public static double Ki         = 0.001;
    public static double Kd         = 0.00001;
    public static double TARGET_RPM = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx shooterLeft  = hardwareMap.get(DcMotorEx.class, "ShooterLeft");
        DcMotorEx shooterRight = hardwareMap.get(DcMotorEx.class, "ShooterRight");
        DcMotorEx intake       = hardwareMap.get(DcMotorEx.class, "Intake");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        double integralSum = 0;
        double lastError   = 0;
        ElapsedTime timer  = new ElapsedTime();

        boolean shooterEnabled  = true;
        boolean intakeRunning   = false;
        boolean prevR1          = false;
        boolean prevL1          = false;

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            // --- R1: toggle shooter PID ---
            boolean currR1 = gamepad1.right_bumper;
            if (currR1 && !prevR1) {
                shooterEnabled = !shooterEnabled;
                if (!shooterEnabled) {
                    integralSum = 0;
                    lastError   = 0;
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    timer.reset();
                }
            }
            prevR1 = currR1;

            // --- L1: toggle intake ---
            boolean currL1 = gamepad1.left_bumper;
            if (currL1 && !prevL1) {
                intakeRunning = !intakeRunning;
                intake.setPower(intakeRunning ? 1.0 : 0.0);
            }
            prevL1 = currL1;

            // --- Shooter PID ---
            double currentTicksPerSec = -shooterLeft.getVelocity();
            double currentRPM         = currentTicksPerSec / TICKS_PER_REV * 60.0;
            double power              = 0;

            if (shooterEnabled) {
                if (TARGET_RPM == 0) {
                    integralSum = 0;
                    lastError   = 0;
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                } else {
                    double targetTicksPerSec = TARGET_RPM * TICKS_PER_REV / 60.0;
                    double error             = targetTicksPerSec - currentTicksPerSec;

                    double derivative = (error - lastError) / timer.seconds();
                    integralSum      += error * timer.seconds();

                    power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
                    power = Math.max(-1.0, Math.min(1.0, power));

                    shooterLeft.setPower(power);
                    shooterRight.setPower(power);

                    lastError = error;
                }
            }

            timer.reset();

            telemetry.addData("shooter", shooterEnabled ? "ON" : "OFF");
            telemetry.addData("target  (RPM)", TARGET_RPM);
            telemetry.addData("current (RPM)", currentRPM);
            telemetry.addData("error   (RPM)", TARGET_RPM - currentRPM);
            telemetry.addData("power", power);
            telemetry.addData("left  encoder (rev)", shooterLeft.getCurrentPosition()  / TICKS_PER_REV);
            telemetry.addData("right encoder (rev)", shooterRight.getCurrentPosition() / TICKS_PER_REV);
            telemetry.addData("intake", intakeRunning ? "ON" : "OFF");
            telemetry.update();
        }

        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        intake.setPower(0);
    }
}