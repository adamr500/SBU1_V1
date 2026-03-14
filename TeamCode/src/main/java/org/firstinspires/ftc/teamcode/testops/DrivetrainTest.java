package org.firstinspires.ftc.teamcode.testops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
//@TeleOp(name = "Drivetrain Test", group = "Test")
public class DrivetrainTest extends LinearOpMode {

    public static double DRIVE_SPEED = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse right side and leftFront
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double axial   = -gamepad1.right_stick_y;  // forward/back
            double lateral =  gamepad1.right_stick_x;  // strafe
            double yaw     =  gamepad1.left_stick_x;   // rotate

            double lf = (axial + lateral + yaw) * DRIVE_SPEED;
            double lb = (axial - lateral + yaw) * DRIVE_SPEED;
            double rf = (axial - lateral - yaw) * DRIVE_SPEED;
            double rb = (axial + lateral - yaw) * DRIVE_SPEED;

            // Normalize so no motor exceeds 1.0
            double max = Math.max(1.0, Math.abs(lf));
            max = Math.max(max, Math.abs(lb));
            max = Math.max(max, Math.abs(rf));
            max = Math.max(max, Math.abs(rb));

            leftFront.setPower(lf / max);
            leftBack.setPower(lb / max);
            rightFront.setPower(rf / max);
            rightBack.setPower(rb / max);

            telemetry.addData("leftFront  power", "%.2f  ticks: %d", lf / max, leftFront.getCurrentPosition());
            telemetry.addData("leftBack   power", "%.2f  ticks: %d", lb / max, leftBack.getCurrentPosition());
            telemetry.addData("rightFront power", "%.2f  ticks: %d", rf / max, rightFront.getCurrentPosition());
            telemetry.addData("rightBack  power", "%.2f  ticks: %d", rb / max, rightBack.getCurrentPosition());
            telemetry.addData("DRIVE_SPEED (dashboard)", DRIVE_SPEED);
            telemetry.update();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}