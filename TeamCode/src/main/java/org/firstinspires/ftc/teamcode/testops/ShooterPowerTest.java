package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
//@TeleOp(name = "Shooter Power Test", group = "Test")
public class ShooterPowerTest extends LinearOpMode {

    static final double TICKS_PER_REV = 28.0;

    public static double TARGET_POWER = 0.5;
    public static double STOPPER_POSITION = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx shooterLeft  = hardwareMap.get(DcMotorEx.class, "ShooterLeft");
        DcMotorEx shooterRight = hardwareMap.get(DcMotorEx.class, "ShooterRight");
        DcMotorEx intake       = hardwareMap.get(DcMotorEx.class, "Intake");
        Servo stopper          = hardwareMap.get(Servo.class, "Stopper");

        stopper.setPosition(STOPPER_POSITION);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setDirection(DcMotorEx.Direction.REVERSE);

        boolean shooterEnabled  = false;
        boolean intakeRunning   = false;
        boolean prevR1          = false;
        boolean prevL1          = false;

        waitForStart();

        while (opModeIsActive()) {
            // --- R1: toggle shooter ---
            boolean currR1 = gamepad1.right_bumper;
            if (currR1 && !prevR1) {
                shooterEnabled = !shooterEnabled;
                if (!shooterEnabled) {
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
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

            // --- Shooter set power ---
            double currentTicksPerSec = shooterLeft.getVelocity();
            double currentRPM         = currentTicksPerSec / TICKS_PER_REV * 60.0;

            if (shooterEnabled) {
                shooterLeft.setPower(TARGET_POWER);
                shooterRight.setPower(TARGET_POWER);
            }

            stopper.setPosition(STOPPER_POSITION);

            telemetry.addData("shooter", shooterEnabled ? "ON" : "OFF");
            telemetry.addData("stopper position", STOPPER_POSITION);
            telemetry.addData("target power", TARGET_POWER);
            telemetry.addData("current (RPM)", currentRPM);
            telemetry.addData("intake", intakeRunning ? "ON" : "OFF");
            telemetry.update();
        }

        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        intake.setPower(0);
    }
}