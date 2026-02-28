package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Turret {

    private final DcMotorEx turret;

    private static final double TICKS_PER_REV    = 145.1;
    private static final double GEAR_RATIO        = 1.0; // external gear ratio if applicable
    private static final double TICKS_PER_DEGREE  = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    // Tune these via FTC Dashboard
    private static final double kP = 10.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0;

    public Turret(HardwareMap hardwareMap) {
        this.turret = hardwareMap.get(DcMotorEx.class, "turret");
        this.turret.setDirection(DcMotorEx.Direction.FORWARD);
//        this.turret.setDirection(DcMotorEx.Direction.REVERSE);
        this.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.turret.setTargetPosition(0);
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(kP, kI, kD, kF));
    }

    public void setAngle(double degrees) {
        int ticks = (int) (degrees * TICKS_PER_DEGREE);
        turret.setTargetPosition(ticks);
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