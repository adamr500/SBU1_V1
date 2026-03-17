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
    public static double HOME_OFFSET_DEG = 40;
    public static double BREAK_POINT_TICKS = 0.0;

    public boolean isReady = false;

    private double lastAngleDeg  = 0;
    private double lastAngleTick = 0;
    private double encoderOffset = 0;

    private double  integralSum = 0;
    private double  lastError   = 0;
    private double  lastPower   = 0;
    private final ElapsedTime timer       = new ElapsedTime();

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
        double targetX   = target.x;
        double targetY   = target.y;
        double currentX  = pose.position.x;
        double currentY  = pose.position.y;

        double fieldAngle       = Math.atan2(targetY - currentY, targetX - currentX);
        double angle            = fieldAngle + pose.heading.toDouble();
        double deg              = Math.toDegrees(angle);
        double normalizedDeg    = ((deg + 180) % 360 + 360) % 360 - 180;
        double limitAdjustedDeg = normalizedDeg + HOME_OFFSET_DEG;

        double rawTicks = limitAdjustedDeg * (TICKS_PER_REV / 360.0);
        double wrappedTicks = ((rawTicks - BREAK_POINT_TICKS) % TICKS_PER_REV + TICKS_PER_REV) % TICKS_PER_REV + BREAK_POINT_TICKS;
        double targetTicks = wrappedTicks + encoderOffset;

        lastAngleDeg  = normalizedDeg;
        lastAngleTick = targetTicks;

        double current = motor.getCurrentPosition();
        double error   = lastAngleTick - current;

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
        encoderOffset = motor.getCurrentPosition() - (HOME_OFFSET_DEG * (TICKS_PER_REV / 360.0));
    }

    public void stop() {
        isReady     = false;
        motor.setPower(0);
        integralSum = 0;
        lastError   = 0;
        lastPower   = 0;
        timer.reset();
    }
}