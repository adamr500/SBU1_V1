package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.hardware.Prism.Color;
import org.firstinspires.ftc.teamcode.hardware.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.hardware.Prism.PrismAnimations;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.hardware.Global;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
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
        Turret turret = new Turret(hardwareMap);
        Shooter shooter = new Shooter();
        shooter.init(hardwareMap);
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        RobotMode mode = RobotMode.MOVING;
        RobotMode lastMode = null;
        boolean lastShooterReady = false;
        long lastLoopTime = System.nanoTime();

        // Corner pose reset
        double RESET_X       =   62.0;
        double RESET_Y       =  -62.0;
        double RESET_HEADING =  90.0;

        boolean circleHeld = false;
        boolean squareHeld = false;

        waitForStart();

        turret.setHomeAtForward();
        shooter.stop();
        turret.stop();

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

            //Alliance selector
            boolean circleNow = gamepad1.circle;
            if (circleNow && !circleHeld) {
                Global.alliance = (Global.alliance == Global.Alliance.RED)
                        ? Global.Alliance.BLUE
                        : Global.Alliance.RED;
            }
            circleHeld = circleNow;

            // Pose reset (drive robot into corner, then press square)
            boolean squareNow = gamepad1.square;
            if (squareNow && !squareHeld) {
                double resetY       = (Global.alliance == Global.Alliance.BLUE) ? -RESET_Y       : RESET_Y;
                double resetHeading = (Global.alliance == Global.Alliance.BLUE) ? -RESET_HEADING : RESET_HEADING;
                drive.localizer.setPose(new Pose2d(RESET_X, resetY, Math.toRadians(resetHeading)));
            }
            squareHeld = squareNow;

            // State actions
            switch (mode) {
                case INTAKING:
                    intake.in();
                    stopper.close();
                    break;

                case SHOOTING:
                    turret.aim(pose);
                    shooter.aim(pose);

                    if (turret.isReady && shooter.isReady) {
                        stopper.open();
                        intake.in();
                    }
                    break;

                case MOVING:
                default:
                    turret.stop();
                    shooter.stop();
                    intake.off();
                    stopper.close();
                    break;
            }

            // LEDs
            boolean shooterReady = turret.isReady && shooter.isReady;
            if (mode != lastMode || (mode == RobotMode.SHOOTING && shooterReady != lastShooterReady)) {
                switch (mode) {
                    case MOVING:
                        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0,
                                new PrismAnimations.Solid(Color.PINK));
                        break;
                    case INTAKING:
                        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0,
                                new PrismAnimations.Solid(Color.BLUE));
                        break;
                    case SHOOTING:
                        if (shooterReady) {
                            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0,
                                    new PrismAnimations.Solid(Color.GREEN));
                        } else {
                            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0,
                                    new PrismAnimations.Solid(Color.ORANGE));
                        }
                        break;
                }
                lastMode = mode;
                lastShooterReady = shooterReady;
            }

            // Drivetrain
            double axial      = gamepad1.right_stick_y;
            double lateral    = gamepad1.right_stick_x;
            double yaw        = -gamepad1.left_stick_x;
            double h = heading - Global.fieldCentricOffset+Math.toRadians(90);
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
           //dashTelemetry.addData("Mode: ", mode);
           //dashTelemetry.addData("Shooter calcRPM", shooter.calcRPM);
            //dashTelemetry.addData("Shooter currentRPM", shooter.currentRPM);
            //dashTelemetry.addData("Shooter calcHoodAngle", shooter.calcHoodAngle);
            //dashTelemetry.addData("Distance from goal", shooter.goalDistance);
            //dashTelemetry.addData("Shooter currentX", shooter.currentX);
            //dashTelemetry.addData("Shooter currentY", shooter.currentY);
            //dashTelemetry.addData("Shooter isReady", shooter.isReady);
            dashTelemetry.addLine("-------------------------------------------------------------");
            dashTelemetry.addData("Turret isReady", turret.isReady);
            dashTelemetry.addData("fieldAngle", turret.fieldAngle);
            dashTelemetry.addData("fieldAngleDeg", turret.fieldAngleDeg);
            dashTelemetry.addData("poseDeg", turret.fieldAngleDeg);
            dashTelemetry.addData("robotAngleDeg", turret.robotAngleDeg);
            dashTelemetry.addData("rawTicks", turret.rawTicks);
            dashTelemetry.addLine("-------------------------------------------------------------");
            dashTelemetry.addLine(String.format(Locale.US, "X: %.2f | Y: %.2f | H: %.2f",
                    pose.position.x,
                    pose.position.y,
                    Math.toDegrees(heading)));
            dashTelemetry.addLine(String.format(Locale.US, "%dHz | %dms", loopHz, loopMs));
            dashTelemetry.update();
        }
    }
}