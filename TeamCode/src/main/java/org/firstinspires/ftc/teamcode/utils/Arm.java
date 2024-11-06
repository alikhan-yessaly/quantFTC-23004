package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

public class Arm {
    private final ServoImplEx arm0Servo;
    private final ServoImplEx arm1Servo;

    public Arm(HardwareMap hardwareMap) {
        arm0Servo = hardwareMap.get(ServoImplEx.class, "arm0");
        arm1Servo = hardwareMap.get(ServoImplEx.class, "arm1");

        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
        arm0Servo.setPwmRange(pwmRange);
        arm1Servo.setPwmRange(pwmRange);
    }

    public void setPosition(double arm0Position, double arm1Position) {
        arm0Servo.setPosition(arm0Position);
        arm1Servo.setPosition(arm1Position);
    }

    public double getArm0Position() {
        return arm0Servo.getPosition();
    }

    public double getArm1Position() {
        return arm1Servo.getPosition();
    }

    public void increaseArm0Position(double increment) {
        double newPosition = Math.min(1.0, arm0Servo.getPosition() + increment); // Ensure the position stays within 0 to 1
        arm0Servo.setPosition(newPosition);
    }

    public void decreaseArm0Position(double decrement) {
        double newPosition = Math.max(0.0, arm0Servo.getPosition() - decrement); // Ensure the position stays within 0 to 1
        arm0Servo.setPosition(newPosition);
    }

    public void increaseArm1Position(double increment) {
        double newPosition = Math.min(1.0, arm1Servo.getPosition() + increment);
        arm1Servo.setPosition(newPosition);
    }

    public void decreaseArm1Position(double decrement) {
        double newPosition = Math.max(0.0, arm1Servo.getPosition() - decrement);
        arm1Servo.setPosition(newPosition);
    }
}
