package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

public class Claw {
    private final ServoImplEx clawServo;

    // Define specific positions for open and closed states
    private static final double GRAB_POSITION = 0.65; // Adjust this to the desired closed position
    private static final double RELEASE_POSITION = 0.3; // Adjust this to the desired open position

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(ServoImplEx.class, "claw");

        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
        clawServo.setPwmRange(pwmRange);
    }

    public void grab() {
        clawServo.setPosition(GRAB_POSITION);
    }

    public void release() {
        clawServo.setPosition(RELEASE_POSITION);
    }

    public double getPosition() {
        return clawServo.getPosition();
    }

    public void setPosition(double position) {
        clawServo.setPosition(position);
    }
}