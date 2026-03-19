package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Turret {

    public static double Kp              = 0.03;
    public static double Ki              = 0;
    public static double Kd              = 0.001;
    public static double TICK_TOLERANCE  = 1.0;

    private final DcMotorEx motor;

    public static double TICKS_PER_REV = 785;
    public static double MAX_TICKS = 698; //  ( 40 - 360 ) / 360 * TICKS_PER_REV
    public static double MIN_TICKS = MAX_TICKS - TICKS_PER_REV;

    public boolean isReady = false;

    //Turret calculation values
    public double fieldAngle = 0;
    public double fieldAngleDeg = 0;
    public double robotAngleDeg = 0;
    public double rawTicks = 0;
    public double normalizedTicks = 0;

    //PID
    private double  integralSum = 0;
    private double  lastError   = 0;
    private double  lastPower   = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public Turret(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "Turret");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        timer.reset();
    }

    public void aim(Pose2d pose) {
        Vector2d target  = Global.currentTarget();

        fieldAngle = Math.atan2(target.y + pose.position.y, target.x - pose.position.x);
        fieldAngleDeg = Math.toDegrees(fieldAngle);
        robotAngleDeg = fieldAngleDeg + Math.toDegrees(pose.heading.toDouble());
        rawTicks = robotAngleDeg * (TICKS_PER_REV / 360.0);
        normalizedTicks = ((rawTicks - MIN_TICKS) % TICKS_PER_REV + TICKS_PER_REV) % TICKS_PER_REV + MIN_TICKS;

        //PID
        double current = motor.getCurrentPosition();
        double error   = normalizedTicks - current;

        isReady = Math.abs(error) <= TICK_TOLERANCE;

        if (isReady) {
            if (lastPower != 0) motor.setPower(0);
            lastError = error;
            lastPower = 0;
            return;
        }

        double dt = timer.seconds();
        timer.reset();

        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        integralSum      += error * dt;
        integralSum       = Math.max(-50, Math.min(50, integralSum));

        double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        power = Math.max(-1.0, Math.min(1.0, power));

        if (Math.abs(power - lastPower) > 0.01) {
            motor.setPower(-power);
        }

        lastError = error;
        lastPower = power;
    }

    public void setHomeAtForward() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    };

    public void stop() {
        isReady     = false;
        motor.setPower(0);
        integralSum = 0;
        lastError   = 0;
        lastPower   = 0;
        timer.reset();
    }
}