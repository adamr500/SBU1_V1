package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intake;

    private static final double INTAKE_IN = 1;
    private static final double INTAKE_OFF = 0;

    public Intake(HardwareMap hardwareMap) {
        this.intake = hardwareMap.get(DcMotor.class, "intake");
        this.intake.setDirection(DcMotor.Direction.FORWARD);
//        this.intake.setDirection(DcMotor.Direction.REVERSE);
    }

    public void in() {
        intake.setPower(INTAKE_IN);
    }

    public void off() {
        intake.setPower(INTAKE_OFF);
    }

    public Action inAction() {
        return packet -> {
            in();
            return false;
        };
    }

    public Action offAction() {
        return packet -> {
            off();
            return false;
        };
    }
}