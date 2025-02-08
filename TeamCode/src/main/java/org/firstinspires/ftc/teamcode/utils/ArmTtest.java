package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmTtest {
    private final DcMotor armT;

    private static final int positionTolerance = 50; // Acceptable error range
    private static double kP = 0.001;  // Proportional gain
    private static double kI = 0.001; // Integral gain
    private static double kD = 0.00001; // Derivative gain

    private static double maxPower = 0.6;

    private static double holdPower = 0.1;

    private int targetPos = 0;
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = System.currentTimeMillis();

    public ArmTtest(HardwareMap hardwareMap, String motorName) {
        armT = hardwareMap.dcMotor.get(motorName);
        armT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPosition(int position) {
        targetPos = position;
        integral = 0; // Reset integral term when setting new position
        lastError = 0;
    }

    public void update() {
        int currentPos = armT.getCurrentPosition();
        int error = targetPos - currentPos;
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0; // Convert ms to seconds
        if (Math.abs(error) > positionTolerance) {
            // PID calculations
            integral += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;
            double power = (kP * error) + (kI * integral) + (kD * derivative);

            // Constrain power
            power = Math.max(-maxPower, Math.min(maxPower, power));

            armT.setPower(power);
        } else {
            // Smooth stop and hold position properly
            integral = 0;  // Reset integral to avoid windup
            lastError = 0;  // Reset derivative influence

            // Apply **just enough power** to hold position
            if (targetPos == 0) {
                armT.setPower(0);  // Don't apply hold power when fully retracted
            } else {
                armT.setPower(holdPower * 0.8);  // Reduce hold power slightly to prevent overshoot
            }
        }

        lastError = error;
        lastTime = currentTime;
    }

    public int getCurrentPosition() {
        return armT.getCurrentPosition();
    }

    public boolean isAtTarget() {
        return Math.abs(targetPos - getCurrentPosition()) <= positionTolerance;
    }

    public void stop() {
        armT.setPower(0);
    }


    // Method to manually adjust PID values
    public void setPID(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }
}