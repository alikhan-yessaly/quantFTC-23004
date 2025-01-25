package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

public class Arm {
    private final ServoImplEx armBServo;
    private final ServoImplEx armTServo;

    public Arm(HardwareMap hardwareMap) {
        armBServo = hardwareMap.get(ServoImplEx.class, "armB");
        armTServo = hardwareMap.get(ServoImplEx.class, "armT");

        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
        armBServo.setPwmRange(pwmRange);
        armTServo.setPwmRange(pwmRange);
    }

    public void setPosition(double armBPosition, double armTPosition) {
        armBServo.setPosition(armBPosition);
        armTServo.setPosition(armTPosition);
    }

    public double getArmBPosition() {
        return armBServo.getPosition();
    }

    public double getArmTPosition() {
        return armTServo.getPosition();
    }

    public void increaseArmBPosition(double increment) {
        double newPosition = Math.min(1.0, armBServo.getPosition() + increment); // Ensure the position stays within 0 to 1
        armBServo.setPosition(newPosition);
    }

    public void decreaseArmBPosition(double decrement) {
        double newPosition = Math.max(0.0, armBServo.getPosition() - decrement); // Ensure the position stays within 0 to 1
        armBServo.setPosition(newPosition);
    }

    public void increaseArmTPosition(double increment) {
        double newPosition = Math.min(1.0, armTServo.getPosition() + increment);
        armTServo.setPosition(newPosition);
    }

    public void decreaseArmTPosition(double decrement) {
        double newPosition = Math.max(0.0, armTServo.getPosition() - decrement);
        armTServo.setPosition(newPosition);
    }
}
