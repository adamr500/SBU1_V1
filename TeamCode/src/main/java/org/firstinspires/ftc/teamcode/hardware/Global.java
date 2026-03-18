package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Global {
    public enum Alliance { RED, BLUE }

    public static Pose2d pose = new Pose2d(0, 0, 0);
    public static double fieldCentricOffset = 0; // set by auto to START_POSE heading
    public static Alliance alliance = Alliance.BLUE;

    public static Vector2d RED_TARGET  = new Vector2d(-60, 60);
    public static Vector2d BLUE_TARGET = new Vector2d(-60, -60);

    public static Vector2d currentTarget() {
        return alliance == Alliance.RED ? RED_TARGET : BLUE_TARGET;
    }
}