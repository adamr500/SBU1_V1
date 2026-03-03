package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Turret {

    private final DcMotorEx turret;

    private static final double TICKS_PER_REV    = 145.1;
    private static final double GEAR_RATIO        = 97.0 / 18.0;
    private static final double TICKS_PER_DEGREE  = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
    private static final double SOFT_LIMIT        = 270.0; // max degrees either side of forward

    // Tune these via FTC Dashboard
    private static final double kP = 10.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0;

    public Turret(HardwareMap hardwareMap) {
        this.turret = hardwareMap.get(DcMotorEx.class, "turret");
        this.turret.setDirection(DcMotorEx.Direction.FORWARD);
        this.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.turret.setTargetPosition(0);
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(kP, kI, kD, kF));
    }

    /**
     * Normalises degrees to the equivalent angle within ±SOFT_LIMIT.
     * Picks whichever of {degrees, degrees±360} is in range and closest to
     * the current position — this is the "wrap-around" flip when a limit is hit.
     */
    private double constrainAngle(double degrees) {
        // Normalise to (-180, 180]
        degrees %= 360;
        if (degrees >  180) degrees -= 360;
        if (degrees < -180) degrees += 360;

        double current  = getCurrentDegrees();
        double best     = Double.NaN;
        double bestDist = Double.MAX_VALUE;

        for (double candidate : new double[]{ degrees, degrees - 360, degrees + 360 }) {
            if (candidate >= -SOFT_LIMIT && candidate <= SOFT_LIMIT) {
                double dist = Math.abs(candidate - current);
                if (dist < bestDist) {
                    bestDist = dist;
                    best = candidate;
                }
            }
        }

        // Safety clamp — shouldn't be reached with a ±270 limit
        if (Double.isNaN(best)) best = Math.max(-SOFT_LIMIT, Math.min(SOFT_LIMIT, degrees));
        return best;
    }

    public void setAngle(double degrees) {
        double constrained = constrainAngle(degrees);
        turret.setTargetPosition((int) (constrained * TICKS_PER_DEGREE));
        turret.setPower(1.0);
    }

    public boolean isAtTarget() {
        return !turret.isBusy();
    }

    public double getCurrentDegrees() {
        return turret.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public double getTargetDegrees() {
        return turret.getTargetPosition() / TICKS_PER_DEGREE;
    }

    // Call every loop in teleop, passing the angle from ShotCalculator
    public void update(double targetDegrees) {
        setAngle(targetDegrees);
    }

    // Runs until the turret reaches the target angle, then completes
    public Action aimAction(double degrees) {
        return packet -> {
            setAngle(degrees);
            return turret.isBusy();
        };
    }
}