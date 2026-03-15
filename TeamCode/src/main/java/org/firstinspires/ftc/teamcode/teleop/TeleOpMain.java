package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.hardware.Global;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Stopper;
import org.firstinspires.ftc.teamcode.hardware.Turret;

import java.util.Locale;

@TeleOp(name = "TeleOpMain", group = "TeleOp")
public class TeleOpMain extends LinearOpMode {

    enum RobotMode { MOVING, INTAKING, SHOOTING }

    @Override
    public void runOpMode() {
        Telemetry dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, Global.pose);
        Intake intake = new Intake(hardwareMap);
        Stopper stopper = new Stopper(hardwareMap);
        Turret turret = new Turret(hardwareMap, dashTelemetry);

        RobotMode mode = RobotMode.MOVING;
        long lastLoopTime = System.nanoTime();

        waitForStart();

        turret.setHomeAtForward();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            Pose2d pose = drive.localizer.getPose();
            double heading = pose.heading.toDouble();

            // State change
            if (gamepad1.left_bumper) {
                mode = RobotMode.INTAKING;
            } else if (gamepad1.right_bumper) {
                mode = RobotMode.SHOOTING;
            } else {
                mode = RobotMode.MOVING;
            }

            // State actions
            switch (mode) {
                case INTAKING:
                    intake.in();
                    stopper.close();
                    break;

                case SHOOTING:
                    turret.aim(pose);
                    break;

                case MOVING:
                default:
                    turret.stop();
                    intake.off();
                    stopper.close();
                    break;
            }

            // Drivetrain
            double axial      = -gamepad1.right_stick_y;
            double lateral    = -gamepad1.right_stick_x;
            double yaw        = -gamepad1.left_stick_x;
            double h = heading - Global.fieldCentricOffset;
            double rotAxial   =  axial * Math.cos(h) + lateral * Math.sin(h);
            double rotLateral = -axial * Math.sin(h) + lateral * Math.cos(h);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotAxial, rotLateral), yaw));

            // Loop time
            long now = System.nanoTime();
            long loopNs = now - lastLoopTime;
            lastLoopTime = now;
            long loopMs = loopNs / 1_000_000;
            long loopHz = loopNs > 0 ? 1_000_000_000L / loopNs : 0;

            // Telemetry
            dashTelemetry.addData("Mode: ", mode);
            turret.telemetry(pose);
            dashTelemetry.addLine(String.format(Locale.US, "X: %.2f | Y: %.2f | H: %.2f",
                    pose.position.x,
                    pose.position.y,
                    Math.toDegrees(heading)));
            dashTelemetry.addLine(String.format(Locale.US, "%dHz | %dms", loopHz, loopMs));
            dashTelemetry.update();
        }
    }
}