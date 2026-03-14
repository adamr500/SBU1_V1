package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.RobotState;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@Autonomous(name = "AutoMain", group = "Autonomous")
public class AutoMain extends LinearOpMode {

    // Starting pose — update this to match where the robot is placed on the field
    private static final Pose2d START_POSE = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);

        waitForStart();

        // TODO: add actions and pathing here
        sleep(1000);

        RobotState.pose = drive.localizer.getPose();
    }
}