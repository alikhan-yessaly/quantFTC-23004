package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

public class LiftUp {
    private final ServoImplEx liftUpServo;
    private final ServoImplEx liftUpLServo;

    public LiftUp(HardwareMap hardwareMap) {
        liftUpServo = hardwareMap.get(ServoImplEx.class, "liftUp");
        liftUpLServo = hardwareMap.get(ServoImplEx.class, "liftUpL");

        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
        liftUpServo.setPwmRange(pwmRange);
        liftUpLServo.setPwmRange(pwmRange);

        liftUpServo.setDirection(ServoImplEx.Direction.REVERSE);
    }

    public void setPosition(double position) {
        liftUpServo.setPosition(position);
        liftUpLServo.setPosition(position);
    }

    public double getPosition() {
        return liftUpServo.getPosition();
    }

    public void raise(double increment) {
        double newPosition = Math.min(1.0, liftUpServo.getPosition() + increment);
        liftUpServo.setPosition(newPosition);
        liftUpLServo.setPosition(newPosition);
    }

    public void lower(double decrement) {
        double newPosition = Math.max(0.0, liftUpServo.getPosition() - decrement);
        liftUpServo.setPosition(newPosition);
        liftUpLServo.setPosition(newPosition);
    }
}