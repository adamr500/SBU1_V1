package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter {

    static final double TICKS_PER_REV = 28.0;

    public static double Kf            = 0.00018;  // feedforward — tune this FIRST
    public static double Kp            = 0.01;
    public static double Ki            = 0.001;
    public static double Kd            = 0.00001;
    public static double MAX_INTEGRAL  = 500;       // integral windup cap
    public static double RPM_TOLERANCE = 75;

    public boolean isReady       = false;
    public double  calcRPM       = 0;
    public double  calcHoodAngle = 0;
    public double  currentRPM    = 0;

    public double goalDistance = 0;
    public double currentX     = 0;
    public double currentY     = 0;


    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private Servo hoodLeft;
    private Servo hoodRight;

    private double integralSum = 0;
    private double lastError   = 0;
    private final ElapsedTime timer = new ElapsedTime();


    public void init(HardwareMap hardwareMap) {
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "ShooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "ShooterRight");

        hoodLeft  = hardwareMap.get(Servo.class, "hoodLeft");
        hoodRight = hardwareMap.get(Servo.class, "hoodRight");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);

        timer.reset();
    }

    public void aim(Pose2d pose) {
        // Distance to goal
        Vector2d target = Global.currentTarget();
        currentX = pose.position.x;
        currentY = -pose.position.y;
        double dX = Math.abs(target.x - currentX);
        double dY = Math.abs(target.y - currentY);
        goalDistance = Math.hypot(dX, dY);

        // Target RPM and hood angle from distance
        calcRPM       = Math.max(0, Math.min(5000, 0.000852363 * Math.pow(goalDistance, 3) - 0.208377 * Math.pow(goalDistance, 2) + 26.2241 * goalDistance + 1827.07895));
        calcHoodAngle = Math.max(0.4, Math.min(0.63, -(1.09516e-7) * Math.pow(goalDistance, 3) + 0.0000583541 * Math.pow(goalDistance, 2) - 0.00771199 * goalDistance + 0.797839));

        // Average both motor velocities for better accuracy
        double leftVel  = Math.abs(shooterLeft.getVelocity());
        double rightVel = Math.abs(shooterRight.getVelocity());
        currentRPM = ((leftVel + rightVel) / 2.0) / TICKS_PER_REV * 60.0;

        // PIDF control
        double targetTicksPerSec  = calcRPM    * TICKS_PER_REV / 60.0;
        double currentTicksPerSec = currentRPM * TICKS_PER_REV / 60.0;
        double error              = targetTicksPerSec - currentTicksPerSec;

        double dt         = timer.seconds();
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;  // guard against dt=0
        integralSum      += error * dt;
        integralSum       = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));  // clamp windup

        double feedforward = Kf * targetTicksPerSec;
        double power = Math.max(-1.0, Math.min(1.0,
                feedforward + (Kp * error) + (Ki * integralSum) + (Kd * derivative)));

        shooterLeft.setPower(power);
        shooterRight.setPower(power);

        hoodRight.setPosition(calcHoodAngle);
        hoodLeft.setPosition(1 - calcHoodAngle);

        lastError = error;
        timer.reset();

        isReady = Math.abs(currentRPM - calcRPM) < RPM_TOLERANCE;
    }


    public void stop() {
        isReady     = false;
        integralSum = 0;
        lastError   = 0;
        timer.reset();
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }
}