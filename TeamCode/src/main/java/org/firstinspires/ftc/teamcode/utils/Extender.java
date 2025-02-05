package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Extender {
    private final DcMotor extender;

    private static final int positionTolerance = 20; // Acceptable error range
    private static double kP = 0.001;  // Proportional gain
    private static double kI = 0.001; // Integral gain
    private static double kD = 0.00001; // Derivative gain

    private static double maxPower = 0.4;

    private static double holdPower = 0.1;

    private int targetPos = 0;
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = System.currentTimeMillis();

    public Extender(HardwareMap hardwareMap, String motorName) {
        extender = hardwareMap.dcMotor.get(motorName);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPosition(int position) {
        targetPos = position;
        integral = 0; // Reset integral term when setting new position
        lastError = 0;
    }

    public void update() {
        int currentPos = extender.getCurrentPosition();
        int error = targetPos - currentPos;
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0;

        if (isAtTarget()) {
            extender.setPower(0);
            extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            integral = 0;
            lastError = 0;
            return;
        }
        if(Math.abs(error) < positionTolerance){
            integral = 0;
        }
        else{
            integral += error * deltaTime;
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
        return -extender.getCurrentPosition();
    }

    public boolean isAtTarget() {
        return Math.abs(targetPos - getCurrentPosition()) <= positionTolerance;
    }


    public void stop() {
        extender.setPower(0);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    // Method to manually adjust PID values
    public void setPID(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }
}