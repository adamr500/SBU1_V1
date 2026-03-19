package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Global;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@Autonomous(name = "AutoMain", group = "Autonomous")
public class AutoMain extends LinearOpMode {

    private static final Pose2d CLOSE_START_RED = new Pose2d(-42, 52, Math.toRadians(-90));
    private static final Pose2d CLOSE_START_BLUE = new Pose2d(-42, 52, Math.toRadians(90));

    private static final Pose2d FAR_START_RED   = new Pose2d(64, 18, Math.toRadians(180));
    private static final Pose2d FAR_START_BLUE   = new Pose2d(64, -18, Math.toRadians(180));


    private static final Pose2d START_POSE = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() {
        Global.fieldCentricOffset = START_POSE.heading.toDouble();
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);

        // ── Init phase: press X to toggle alliance ──────────────────────────
        boolean xHeld = false;
        while (!isStarted() && !isStopRequested()) {
            boolean xNow = gamepad1.circle;
            if (xNow && !xHeld) {
                Global.alliance = (Global.alliance == Global.Alliance.RED)
                        ? Global.Alliance.BLUE
                        : Global.Alliance.RED;
            }
            xHeld = xNow;

            telemetry.addData("Alliance (Circle to toggle)", Global.alliance);
            telemetry.update();
        }
        // ────────────────────────────────────────────────────────────────────

        waitForStart();

        // TODO: add actions and pathing here
        sleep(20);

        Global.pose = drive.localizer.getPose();
    }
}