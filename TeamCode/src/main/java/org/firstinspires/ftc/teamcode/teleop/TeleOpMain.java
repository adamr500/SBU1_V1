package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Prism.Color;
import org.firstinspires.ftc.teamcode.hardware.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.hardware.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Stopper;

import java.util.Locale;

@TeleOp(name = "TeleOpMain", group = "TeleOp")
public class TeleOpMain extends LinearOpMode {

    enum RobotMode { MOVING, INTAKING, SHOOTING }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, RobotState.pose);
        PinpointLocalizer pinpoint = (PinpointLocalizer) drive.localizer;
        Intake intake = new Intake(hardwareMap);
        Stopper stopper = new Stopper(hardwareMap);
        Shooter shooter = new Shooter();
        shooter.init(hardwareMap);
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        // Turret: hold position using BRAKE zero-power behaviour
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setPower(0);

        RobotMode mode = RobotMode.MOVING;
        RobotMode lastMode = null;

        // Set initial LED colour before start
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0,
                new PrismAnimations.Solid(Color.ORANGE));

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            // --- State transitions ---
            if (gamepad1.left_bumper) {
                mode = RobotMode.INTAKING;
            } else if (gamepad1.right_bumper) {
                mode = RobotMode.SHOOTING;
            } else {
                mode = RobotMode.MOVING;
            }

            // --- LED update (only on mode change) ---
            if (mode != lastMode) {
                if (lastMode == RobotMode.SHOOTING) {
                    shooter.stop();
                }
                switch (mode) {
                    case INTAKING:
                        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0,
                                new PrismAnimations.Solid(Color.GREEN));
                        break;
                    case SHOOTING:
                        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0,
                                new PrismAnimations.Solid(Color.PINK));
                        shooter.update(Shooter.TARGET_RPM);
                        shooter.setHoodPosition(Shooter.HOOD_POSITION);
                        sleep(750);
                        break;
                    case MOVING:
                    default:
                        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0,
                                new PrismAnimations.Solid(Color.ORANGE));
                        break;
                }
                lastMode = mode;
            }

            // --- State actions ---
            double shooterPower = 0;
            switch (mode) {
                case INTAKING:
                    intake.in();
                    stopper.close();
                    break;

                case SHOOTING:
                    intake.in();
                    stopper.open();
                    shooterPower = shooter.update(Shooter.TARGET_RPM);
                    shooter.setHoodPosition(Shooter.HOOD_POSITION);
                    break;

                case MOVING:
                default:
                    intake.off();
                    stopper.close();
                    break;
            }

            // --- Drive (always active) ---
            double heading    = drive.localizer.getPose().heading.toDouble();
            double axial      = -gamepad1.right_stick_y;
            double lateral    = -gamepad1.right_stick_x;
            double yaw        = -gamepad1.left_stick_x;
            double rotAxial   =  axial * Math.cos(heading) + lateral * Math.sin(heading);
            double rotLateral = -axial * Math.sin(heading) + lateral * Math.cos(heading);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotAxial, rotLateral), yaw));

            // --- Telemetry ---
            telemetry.addData("Mode", mode);
            telemetry.addLine(String.format(Locale.US, "X: %.2f | Y: %.2f | H: %.2f",
                    drive.localizer.getPose().position.x,
                    drive.localizer.getPose().position.y,
                    Math.toDegrees(heading)));
            if (mode == RobotMode.SHOOTING) {
                telemetry.addData("Shooter target (RPM)", Shooter.TARGET_RPM);
                telemetry.addData("Shooter current (RPM)", shooter.getCurrentRPM());
                telemetry.addData("Shooter error (RPM)", Shooter.TARGET_RPM - shooter.getCurrentRPM());
                telemetry.addData("Shooter power", shooterPower);
                telemetry.addData("Shooter ready", shooter.isReady);
                telemetry.addData("Hood position", Shooter.HOOD_POSITION);
            }
            telemetry.update();
        }
    }
}