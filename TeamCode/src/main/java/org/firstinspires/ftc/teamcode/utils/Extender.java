package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.GoBildaPinpointDriver;

public class Extender {
    private final DcMotor extender;

    private static final int positionTolerance = 50; // Acceptable error range
    private static double kP = 0.0005;  // Proportional gain
    private static double kI = 0.0003; // Integral gain
    private static double kD = 0.00001; // Derivative gain

    private static double maxPower = 1.0;
    private static double holdPower = 0.1;

    private int targetPos = 0;
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = System.currentTimeMillis();

    public Extender(HardwareMap hardwareMap, String motorName) {
        extender = hardwareMap.dcMotor.get(motorName);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setDirection(DcMotorSimple.Direction.REVERSE);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Use encoder for proper PID// Prevent free movement
    }

    public void setPosition(int position) {
        targetPos = position;
        integral = 0; // Reset integral term when setting new position
        lastError = 0;
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
        int currentPos = extender.getCurrentPosition();
        int error = targetPos - currentPos;
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0; // Convert ms to seconds

        if (isAtTarget()) {
            extender.setPower(0);
            extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);// Stop all power when at target
            integral = 0;          // Reset integral to avoid windup
            lastError = 0;         // Reset derivative influence
            return;                // Exit the function to prevent further updates
        }

        // Prevent integral windup when the error is small
        if (Math.abs(error) > positionTolerance * 2) {
            integral += error * deltaTime;
        } else {
            integral = 0;
        }

        // PID calculations
        double derivative = (error - lastError) / (deltaTime + 0.001);
        double power = (kP * error) + (kI * integral) + (kD * derivative);

        // Constrain power
        power = Math.max(-maxPower, Math.min(maxPower, power));

        extender.setPower(power);

        lastError = error;
        lastTime = currentTime;
    }

    public int getCurrentPosition() {
        return extender.getCurrentPosition();
    }

    public boolean isAtTarget() {
        return Math.abs(targetPos - getCurrentPosition()) <= positionTolerance;
    }

    public void stop() {
        extender.setPower(0);
    }

    public void setPID(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }
}