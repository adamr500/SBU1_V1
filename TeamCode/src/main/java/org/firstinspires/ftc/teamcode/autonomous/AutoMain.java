package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Global;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Turret;

@Autonomous(name = "AutoMain", group = "Autonomous")
public class AutoMain extends LinearOpMode {

    private static final Pose2d CLOSE_START_RED = new Pose2d(-42, 52, Math.toRadians(-90));
    private static final Pose2d CLOSE_START_BLUE = new Pose2d(-42, -52, Math.toRadians(90));

    private static final Pose2d FAR_START_RED   = new Pose2d(64, 18, Math.toRadians(180));
    private static final Pose2d FAR_START_BLUE   = new Pose2d(64, -18, Math.toRadians(180));


    private static final Pose2d START_POSE = new Pose2d(0, 0, 0);

    enum Zone { CLOSE, FAR }

    Turret turret;

    @Override
    public void runOpMode() {

        turret = new Turret(hardwareMap);

        Zone zone = Zone.CLOSE;

        // ── Init phase ───────────────────────────────────────────────────────
        turret.resetEncoders();

        boolean circleHeld = false;
        boolean triangleHeld = false;
        while (!isStarted() && !isStopRequested()) {
            boolean circleNow = gamepad1.circle;
            if (circleNow && !circleHeld) {
                Global.alliance = (Global.alliance == Global.Alliance.RED)
                        ? Global.Alliance.BLUE
                        : Global.Alliance.RED;
            }
            circleHeld = circleNow;

            boolean triangleNow = gamepad1.triangle;
            if (triangleNow && !triangleHeld) {
                zone = (zone == Zone.CLOSE) ? Zone.FAR : Zone.CLOSE;
            }
            triangleHeld = triangleNow;

            telemetry.addData("Alliance  (Circle to toggle)",   Global.alliance);
            telemetry.addData("Zone    (Triangle to toggle)",   zone);
            telemetry.update();
        }
        // ────────────────────────────────────────────────────────────────────

        Pose2d startPose;
        if (Global.alliance == Global.Alliance.RED) {
            startPose = (zone == Zone.CLOSE) ? CLOSE_START_RED : FAR_START_RED;
        } else {
            startPose = (zone == Zone.CLOSE) ? CLOSE_START_BLUE : FAR_START_BLUE;
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        waitForStart();

        // TODO: add actions and pathing here
        sleep(20);
        Global.pose = drive.localizer.getPose();
        sleep(20);
    }
}