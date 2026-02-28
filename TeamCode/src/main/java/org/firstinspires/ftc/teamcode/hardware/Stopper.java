package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Stopper {

    private final Servo stopper;

    private static final double STOPPER_CLOSED = 0.7;
    private static final double STOPPER_OPEN = 0.3;

    public Stopper(HardwareMap hardwareMap) {
        this.stopper = hardwareMap.get(Servo.class, "stopper");
        this.stopper.setDirection(Servo.Direction.FORWARD);
    }

    public void close() {
        stopper.setPosition(STOPPER_CLOSED);
    }

    public void open() {
        stopper.setPosition(STOPPER_OPEN);
    }

    public Action closeAction() {
        return packet -> {
            close();
            return false;
        };
    }

    public Action openAction() {
        return packet -> {
            open();
            return false;
        };
    }
}