package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

public class Extender {
    private final ServoImplEx extenderRServo;
    private final ServoImplEx extenderLServo;

    public Extender(HardwareMap hardwareMap) {
        extenderRServo = hardwareMap.get(ServoImplEx.class, "extendR");
        extenderLServo = hardwareMap.get(ServoImplEx.class, "extendL");

        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
        extenderRServo.setPwmRange(pwmRange);
        extenderLServo.setPwmRange(pwmRange);

        extenderLServo.setDirection(Servo.Direction.REVERSE);

    }

    public void setPosition(double position) {
        extenderRServo.setPosition(position);
        extenderLServo.setPosition(position);
    }

    public double getExtendRPosition() {
        return extenderRServo.getPosition();
    }

    public double getExtendLPosition(){
        return extenderLServo.getPosition();
    }

    public void raise(double increment) {
        double newPosition = Math.min(1.0, extenderRServo.getPosition() + increment);
        extenderRServo.setPosition(newPosition);
        extenderLServo.setPosition(newPosition);
    }

    public void lower(double decrement) {
        double newPosition = Math.max(0.0, extenderRServo.getPosition() - decrement);
        extenderRServo.setPosition(newPosition);
        extenderLServo.setPosition(newPosition);
    }
}