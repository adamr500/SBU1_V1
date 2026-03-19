package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Global {
    public enum Alliance { RED, BLUE }

    public static final double ROBOT_WIDTH = 12.1875;
    public static final double ROBOT_LONGTH = 14.375;


    public static Pose2d pose = new Pose2d(0, 0, 0);
    public static double fieldCentricOffset = 0; // DO NOT TOUCH! set by autonomous for auto -> teleop pose carry over
    public static Alliance alliance = Alliance.BLUE;

    public static Vector2d RED_TARGET  = new Vector2d(-60, 53);
    public static Vector2d BLUE_TARGET = new Vector2d(-60, -53);

    public static Vector2d currentTarget() {
        return alliance == Alliance.RED ? BLUE_TARGET : RED_TARGET; //robot was tracking opposite goal so i just swapped RED and BLUE
    }
}