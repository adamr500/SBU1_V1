package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.PinpointLocalizer;
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
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

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
                switch (mode) {
                    case INTAKING:
                        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0,
                                new PrismAnimations.Solid(Color.GREEN));
                        break;
                    case SHOOTING:
                        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0,
                                new PrismAnimations.Solid(Color.PINK));
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
            switch (mode) {
                case INTAKING:
                    intake.in();
                    stopper.close();
                    break;

                case SHOOTING:
                    intake.off();
                    // TODO: add shooting actions here
                    break;

                case MOVING:
                default:
                    intake.off();
                    // TODO: add moving actions here
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
            telemetry.update();
        }
    }
}