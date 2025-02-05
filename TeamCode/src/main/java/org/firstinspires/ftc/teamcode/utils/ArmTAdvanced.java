package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmTAdvanced {
    private DcMotorEx armMotor;
    private static final int ENCODER_TICKS_PER_REV = 8000; // External encoder resolution
    private static final double GEAR_RATIO = 1.0; // Direct drive
    private static final double MAX_MOTOR_POWER = 1.0;

    // PID Gains (To be tuned)
    private double Kp_pos = 0.01, Ki_pos = 0.001, Kd_pos = 0.05;
    private double Kp_vel = 1.0, Ki_vel = 0.1, Kd_vel = 0.2;
    private double Kp_current = 0.5, Ki_current = 0.05, Kd_current = 0.1;

    // Gravity Compensation Parameters
    private static final double ARM_MASS = 1.0;  // kg
    private static final double ARM_LENGTH = 0.2; // m (to center of mass)
    private static final double GRAVITY = 9.81;   // m/s²

    // PID Memory
    private double prevErrorPos = 0, integralPos = 0;
    private double prevErrorVel = 0, integralVel = 0;
    private double prevErrorCurrent = 0, integralCurrent = 0;
    private double prevVelocity = 0, prevTorque = 0;
    private double lowPassAlpha = 0.1;  // Smoothing factor for filtering

    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    // Target position (setpoint) in radians
    private double targetPosition = Math.toRadians(45);  // Default: 45 degrees

    public ArmTAdvanced(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armT");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Set the desired arm angle in degrees
    public void setTargetAngle(double degrees) {
        targetPosition = Math.toRadians(degrees);
    }

    // ✅ New method: Get the target angle in degrees
    public double getTargetAngle() {
        return Math.toDegrees(targetPosition);
    }

    public void update() {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        if (dt < 0.01) return; // Avoid excessive updates

        // Read Encoder (Convert to Radians)
        int encoderCounts = armMotor.getCurrentPosition();
        double currentPosition = (encoderCounts / (double) ENCODER_TICKS_PER_REV) * (2 * Math.PI);

        // ----------------------
        // 1. Position PID Control
        // ----------------------
        double errorPos = targetPosition - currentPosition;
        integralPos += errorPos * dt;
        double derivativePos = (errorPos - prevErrorPos) / dt;
        double velocityCommand = Kp_pos * errorPos + Ki_pos * integralPos + Kd_pos * derivativePos;
        prevErrorPos = errorPos;

        // Low-pass filter on velocity
        double filteredVelocity = lowPassAlpha * velocityCommand + (1 - lowPassAlpha) * prevVelocity;

        // ----------------------
        // 2. Velocity PID Control
        // ----------------------
        double errorVel = filteredVelocity - prevVelocity;
        integralVel += errorVel * dt;
        double derivativeVel = (errorVel - prevErrorVel) / dt;
        double torqueCommand = Kp_vel * errorVel + Ki_vel * integralVel + Kd_vel * derivativeVel;
        prevErrorVel = errorVel;
        prevVelocity = filteredVelocity;

        // ----------------------
        // 3. Gravity Compensation
        // ----------------------
        double gravityTorque = ARM_MASS * GRAVITY * ARM_LENGTH * Math.sin(currentPosition);
        torqueCommand += gravityTorque; // Add gravity compensation
        // ----------------------
        // 4. Current PID Control (Final Output)
        // ----------------------
        double filteredTorque = lowPassAlpha * torqueCommand + (1 - lowPassAlpha) * prevTorque;
        double errorCurrent = filteredTorque - prevTorque;
        integralCurrent += errorCurrent * dt;
        double derivativeCurrent = (errorCurrent - prevErrorCurrent) / dt;
        double pwmOutput = Kp_current * errorCurrent + Ki_current * integralCurrent + Kd_current * derivativeCurrent;
        prevErrorCurrent = errorCurrent;
        prevTorque = filteredTorque;

        // ----------------------
        // 5. Apply Motor Power (Limit to Safe Range)
        // ----------------------
        double motorPower = Math.max(-MAX_MOTOR_POWER, Math.min(MAX_MOTOR_POWER, pwmOutput));
        armMotor.setPower(motorPower);

        lastTime = currentTime;
    }

    // --------------------
    // AUTOTUNING FUNCTION
    // --------------------
    public void autoTune() {
        System.out.println("Starting PID autotuning...");

        // Step 1: Tune Current PID (Lowest Level)
        System.out.println("Tuning Current PID...");
        double[] tunedCurrentPID = tunePIDLoop(0.1, 0.5, 0.01);
        Kp_current = tunedCurrentPID[0];
        Ki_current = tunedCurrentPID[1];
        Kd_current = tunedCurrentPID[2];
        System.out.printf("Optimized Current PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp_current, Ki_current, Kd_current);

        // Step 2: Tune Velocity PID (Using Tuned Current PID)
        System.out.println("Tuning Velocity PID...");
        double[] tunedVelocityPID = tunePIDLoop(0.5, 1.0, 0.05);
        Kp_vel = tunedVelocityPID[0];
        Ki_vel = tunedVelocityPID[1];
        Kd_vel = tunedVelocityPID[2];
        System.out.printf("Optimized Velocity PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp_vel, Ki_vel, Kd_vel);

        // Step 3: Tune Position PID (Using Tuned Velocity PID)
        System.out.println("Tuning Position PID...");
        double[] tunedPositionPID = tunePIDLoop(1.0, 2.0, 0.1);
        Kp_pos = tunedPositionPID[0];
        Ki_pos = tunedPositionPID[1];
        Kd_pos = tunedPositionPID[2];
        System.out.printf("Optimized Position PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp_pos, Ki_pos, Kd_pos);

        System.out.println("PID Autotuning Complete!");
    }

    // --------------------
    // PID LOOP TUNING FUNCTION
    // --------------------
    private double[] tunePIDLoop(double initialKp, double maxKp, double stepSize) {
        double bestKp = initialKp, bestKi = 0.01, bestKd = 0.05;
        double minError = Double.MAX_VALUE;
        for (double Kp = initialKp; Kp <= maxKp; Kp += stepSize) {
            for (double Ki = 0.01; Ki <= 0.2; Ki += 0.01) {
                for (double Kd = 0.05; Kd <= 0.5; Kd += 0.05) {
                    double error = testPIDResponse(Kp, Ki, Kd);
                    if (error < minError) {
                        minError = error;
                        bestKp = Kp;
                        bestKi = Ki;
                        bestKd = Kd;
                    }
                }
            }
        }

        return new double[]{bestKp, bestKi, bestKd};
    }

    // --------------------
    // STEP RESPONSE TEST FUNCTION
    // --------------------
    private double testPIDResponse(double Kp, double Ki, double Kd) {
        double testTarget = Math.toRadians(30); // Move to 30° target
        double currentPosition, errorSum = 0;
        long startTime = System.currentTimeMillis();
        long duration = 2000; // 2 seconds for response test

        while (System.currentTimeMillis() - startTime < duration) {
            int encoderCounts = armMotor.getCurrentPosition();
            currentPosition = (encoderCounts / (double) ENCODER_TICKS_PER_REV) * (2 * Math.PI);

            double errorPos = testTarget - currentPosition;
            double velocityCommand = Kp * errorPos;
            double torqueCommand = Kp * velocityCommand;

            double motorPower = Math.max(-MAX_MOTOR_POWER, Math.min(MAX_MOTOR_POWER, torqueCommand));
            armMotor.setPower(motorPower);

            errorSum += Math.abs(errorPos);
        }

        return errorSum / duration; // Return average error
    }
}