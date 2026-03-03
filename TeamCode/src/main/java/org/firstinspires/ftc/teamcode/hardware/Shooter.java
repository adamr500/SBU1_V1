package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter {

    static final double TICKS_PER_REV = 28.0;

    public static double Kp            = 0.001;
    public static double Ki            = 0.00000325;
    public static double Kd            = 0.0;
    public static double RPM_TOLERANCE = 50.0;

    /** True once the flywheel has reached its target RPM. Read this from your teleop to gate shooting. */
    public boolean isReady = false;

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    private double integralSum = 0;
    private double lastError   = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hardwareMap) {
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "ShooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "ShooterRight");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);

        timer.reset();
    }

    /** Run one PID step toward targetRPM. Call each loop iteration in teleop. Returns motor power applied. */
    public double update(double targetRPM) {
        double targetTicksPerSec  = targetRPM * TICKS_PER_REV / 60.0;
        double currentTicksPerSec = shooterLeft.getVelocity();
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

    public double getCurrentRPM() {
        return shooterLeft.getVelocity() / TICKS_PER_REV * 60.0;
    }

    public boolean atSpeed(double targetRPM) {
        return Math.abs(getCurrentRPM() - targetRPM) < RPM_TOLERANCE;
    }

    public void resetPID() {
        integralSum = 0;
        lastError   = 0;
        timer.reset();
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
            packet.put("shooter/power",      power);
            packet.put("shooter/isReady",    isReady);

            return true; // runs until the action sequence cancels it
        }
    }
}