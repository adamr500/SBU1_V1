package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter {

    static final double TICKS_PER_REV = 28.0;

    public static double Kp                   = 0.001;
    public static double Ki                   = 0.00000325;
    public static double Kd                   = 0.0;
    public static double TARGET_RPM           = 1200;
    public static double HOOD_POSITION        = 0.63;
    public static double RPM_TOLERANCE        = 50.0;
    public static double RPM_AGREEMENT_THRESH = 100.0; // use mean when both readings are within this
    public static double NEAR_ZERO_THRESH     = 50.0;  // treat a reading as "zero" if below this
    public static double VARIANCE_ALPHA       = 0.2;   // EMA smoothing factor for stability estimate

    /** True once the flywheel has reached its target RPM. Read this from your teleop to gate shooting. */
    public boolean isReady = false;

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private Servo hoodLeft;
    private Servo hoodRight;

    private double integralSum = 0;
    private double lastError   = 0;
    private final ElapsedTime timer = new ElapsedTime();

    // RPM sensor fusion state
    private double prevLeftRPM   = 0;
    private double prevRightRPM  = 0;
    private double leftVariance  = 0; // EMA of |delta RPM| — lower = more stable
    private double rightVariance = 0;

    public void init(HardwareMap hardwareMap) {
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "ShooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "ShooterRight");

        hoodLeft  = hardwareMap.get(Servo.class, "hoodLeft");
        hoodRight = hardwareMap.get(Servo.class, "hoodRight");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);

        timer.reset();
    }

    /**
     * Fused RPM reading from both encoders.
     * - Within RPM_AGREEMENT_THRESH of each other → mean of both.
     * - Otherwise → whichever side is more stable (lower EMA variance) and not near zero.
     */
    private double getEffectiveRPM() {
        double leftRPM  = shooterLeft.getVelocity()  / TICKS_PER_REV * 60.0;
        double rightRPM = shooterRight.getVelocity() / TICKS_PER_REV * 60.0;

        // Update EMA variance (stability) estimates
        leftVariance  = VARIANCE_ALPHA * Math.abs(leftRPM  - prevLeftRPM)
                      + (1.0 - VARIANCE_ALPHA) * leftVariance;
        rightVariance = VARIANCE_ALPHA * Math.abs(rightRPM - prevRightRPM)
                      + (1.0 - VARIANCE_ALPHA) * rightVariance;
        prevLeftRPM  = leftRPM;
        prevRightRPM = rightRPM;

        if (Math.abs(leftRPM - rightRPM) <= RPM_AGREEMENT_THRESH) {
            return (leftRPM + rightRPM) / 2.0;
        }

        // Readings disagree — pick the more stable, non-zero side
        boolean leftNearZero  = Math.abs(leftRPM)  < NEAR_ZERO_THRESH;
        boolean rightNearZero = Math.abs(rightRPM) < NEAR_ZERO_THRESH;

        if (leftNearZero  && rightNearZero) return (leftRPM + rightRPM) / 2.0;
        if (leftNearZero)                   return rightRPM;
        if (rightNearZero)                  return leftRPM;

        return (leftVariance <= rightVariance) ? leftRPM : rightRPM;
    }

    /** Run one PID step toward targetRPM. Call each loop iteration in teleop. Returns motor power applied. */
    public double update(double targetRPM) {
        double targetTicksPerSec  = targetRPM * TICKS_PER_REV / 60.0;
        double effectiveRPM       = getEffectiveRPM();
        double currentTicksPerSec = effectiveRPM * TICKS_PER_REV / 60.0;
        double error              = targetTicksPerSec - currentTicksPerSec;

        double dt         = timer.seconds();
        double derivative = (error - lastError) / dt;
        integralSum      += error * dt;

        double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        power = Math.max(-1.0, Math.min(1.0, power));

        shooterLeft.setPower(power);
        shooterRight.setPower(power);

        lastError = error;
        timer.reset();

        isReady = atSpeed(targetRPM);
        return power;
    }

    /** Returns the fused effective RPM used by the PID. */
    public double getCurrentRPM() {
        return getEffectiveRPM();
    }

    public double getLeftRPM() {
        return shooterLeft.getVelocity() / TICKS_PER_REV * 60.0;
    }

    public double getRightRPM() {
        return shooterRight.getVelocity() / TICKS_PER_REV * 60.0;
    }

    public boolean atSpeed(double targetRPM) {
        return Math.abs(getCurrentRPM() - targetRPM) < RPM_TOLERANCE;
    }

    public void resetPID() {
        integralSum   = 0;
        lastError     = 0;
        prevLeftRPM   = 0;
        prevRightRPM  = 0;
        leftVariance  = 0;
        rightVariance = 0;
        timer.reset();
    }

    /** Sets hood angle. position is [0, 1]; the two servos mirror each other. */
    public void setHoodPosition(double position) {
        hoodRight.setPosition(position);
        hoodLeft.setPosition(1.0 - position);
    }

    public void stop() {
        isReady = false;
        resetPID();
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    /**
     * RR Action: PID to targetRPM and complete once within RPM_TOLERANCE.
     * Use sequentially in auto to wait for the flywheel to spin up before shooting.
     */
    public Action spinUpTo(double targetRPM) {
        return new SpinUpAction(targetRPM);
    }

    /**
     * RR Action: PID to targetRPM and hold indefinitely.
     * Use as a parallel action alongside a path so the flywheel stays at speed.
     */
    public Action holdRPM(double targetRPM) {
        return new HoldRPMAction(targetRPM);
    }

    private class SpinUpAction implements Action {
        private final double targetRPM;

        SpinUpAction(double targetRPM) {
            this.targetRPM = targetRPM;
            resetPID();
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            double power      = update(targetRPM);
            double currentRPM = getCurrentRPM();

            packet.put("shooter/targetRPM",  targetRPM);
            packet.put("shooter/currentRPM", currentRPM);
            packet.put("shooter/leftRPM",    getLeftRPM());
            packet.put("shooter/rightRPM",   getRightRPM());
            packet.put("shooter/power",      power);
            packet.put("shooter/isReady",    isReady);

            return !atSpeed(targetRPM); // returns false (done) once at speed
        }
    }

    private class HoldRPMAction implements Action {
        private final double targetRPM;

        HoldRPMAction(double targetRPM) {
            this.targetRPM = targetRPM;
            resetPID();
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            double power      = update(targetRPM);
            double currentRPM = getCurrentRPM();

            packet.put("shooter/targetRPM",  targetRPM);
            packet.put("shooter/currentRPM", currentRPM);
            packet.put("shooter/leftRPM",    getLeftRPM());
            packet.put("shooter/rightRPM",   getRightRPM());
            packet.put("shooter/power",      power);
            packet.put("shooter/isReady",    isReady);

            return true; // runs until the action sequence cancels it
        }
    }
}