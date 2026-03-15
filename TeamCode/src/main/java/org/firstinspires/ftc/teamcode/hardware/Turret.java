package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

@Config
public class Turret {

    public static double Kp = 0.03;
    public static double Ki = 0;
    public static double Kd = 0.001;

    private final Telemetry telemetry;
    private final DcMotorEx motor;

    public static final double TICKS_PER_REV = 882.7;
    public static final double TICKS_PER_DEG = TICKS_PER_REV / 360.0; // 2.4519

    // Degrees from robot forward to the cable break point (positive = break point is to the left)
    public static double BREAK_POINT_DEG = 40.0;

    private double lastAngleDeg  = 0;
    private double lastAngleTick = 0;
    private double encoderOffset = 0;

    private double      integralSum  = 0;
    private double      lastError    = 0;
    private double      lastPower    = 0;
    private final ElapsedTime timer  = new ElapsedTime();

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        motor = hardwareMap.get(DcMotorEx.class, "Turret");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void aim(Pose2d pose) {
        Vector2d target  = Global.currentTarget();
        double targetX   = target.x;
        double targetY   = target.y;
        double currentX  = -pose.position.x;
        double currentY  = pose.position.y;

        double fieldAngle       = Math.atan2(targetY - currentY, targetX - currentX);
        double angle            = fieldAngle + pose.heading.toDouble();
        double deg              = Math.toDegrees(angle);
        double normalizedDeg    = ((deg + 180) % 360 + 360) % 360 - 180;
        double limitAdjustedDeg = normalizedDeg + BREAK_POINT_DEG;
        double targetTicks      = limitAdjustedDeg * TICKS_PER_DEG + encoderOffset;

        // Wrap targetTicks into valid range [0, TICKS_PER_REV]
        // PID will naturally take the shortest path, so this avoids crossing the break point
        while (targetTicks > TICKS_PER_REV) {
            targetTicks -= TICKS_PER_REV;
        }
        while (targetTicks < 0) {
            targetTicks += TICKS_PER_REV;
        }

        lastAngleDeg  = normalizedDeg;
        lastAngleTick = targetTicks;

        double current = motor.getCurrentPosition();
        double error   = lastAngleTick - current;

        if (Math.abs(error) <= 1.0) {
            motor.setPower(0);
            lastError = error;
            lastPower = 0;
            return;
        }

        double dt = timer.seconds();
        timer.reset();

        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        integralSum      += error * dt;

        double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        power = Math.max(-1.0, Math.min(1.0, power));

        motor.setPower(-power);
        lastError = error;
        lastPower = power;
    }

    /** Call when turret is physically pointing straight forward.
     *  Sets the encoder reference so that tick 0 = the cable break point. */
    public void setHomeAtForward() {
        encoderOffset = motor.getCurrentPosition() - (BREAK_POINT_DEG * TICKS_PER_DEG);
    }

    public void stop() {
        motor.setPower(0);
        integralSum = 0;
        lastError   = 0;
        timer.reset();
    }

    public void telemetry(Pose2d pose) {
        Vector2d target = Global.currentTarget();
        double currentX = pose.position.x;
        double currentY = pose.position.y;

        telemetry.addLine(String.format(Locale.US,
                "T: (%.1f, %.1f) | C: (%.1f, %.1f) | A: (%.1f°, %.1ft)",
                target.x, target.y, currentX, currentY, lastAngleDeg, lastAngleTick));

        telemetry.addLine(String.format(Locale.US,
                "Turret PID | cur: %d | tgt: %.1f | pwr: %.3f | break: %.1f°",
                motor.getCurrentPosition(), lastAngleTick, lastPower, BREAK_POINT_DEG));
    }
}