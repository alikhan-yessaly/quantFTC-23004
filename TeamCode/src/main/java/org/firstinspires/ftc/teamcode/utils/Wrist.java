package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

public class Wrist {
    private final ServoImplEx wristServo;

    // Define the preset positions
    private static final double POSITION_1 = 0.2; // Adjust these values as needed
    private static final double POSITION_2 = 0.5;
    private static final double POSITION_3 = 0.8;

    private int positionIndex = 0; // Tracks the current position in the cycle

    public Wrist(HardwareMap hardwareMap) {
        wristServo = hardwareMap.get(ServoImplEx.class, "wrist");

        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
        wristServo.setPwmRange(pwmRange);

        // Initialize to the first position
        setPosition(POSITION_1);
    }

    // Method to cycle through positions
    public void cyclePosition() {
        positionIndex = (positionIndex + 1) % 3; // Cycle between 0, 1, and 2
        switch (positionIndex) {
            case 0:
                setPosition(POSITION_1);
                break;
            case 1:
                setPosition(POSITION_2);
                break;
            case 2:
                setPosition(POSITION_3);
                break;
        }
    }

    public void setPosition(double position) {
        wristServo.setPosition(position);
    }

    public double getPosition() {
        return wristServo.getPosition();
    }
}
