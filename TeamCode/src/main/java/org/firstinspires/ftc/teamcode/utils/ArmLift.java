package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmLift {
    private final DcMotor armLift1;
    private final DcMotor armLift2;

    // Define positions and tolerance
    private static final int DOWN_POSITION = 0;
    private static final int UP_POSITION = 2230;
    private static final int POSITION_TOLERANCE = 20;  // Tolerance for "close enough" to target position
    private static final double MAX_POWER = 1.0;

    public ArmLift(HardwareMap hardwareMap) {
        armLift1 = hardwareMap.dcMotor.get("lift1");
        armLift2 = hardwareMap.dcMotor.get("lift2");

        // Reset encoders and set modes
        armLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLift1.setDirection(DcMotor.Direction.REVERSE);

        armLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set the arm lift to a specified position
    public void setPosition(int position) {
        armLift1.setTargetPosition(position);
        armLift2.setTargetPosition(position);
        armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift1.setPower(MAX_POWER);
        armLift2.setPower(MAX_POWER);
    }

    // Move the arm lift all the way down
    public void moveDown() {
        setPosition(DOWN_POSITION);
    }

    // Move the arm lift all the way up
    public void moveUp() {
        setPosition(UP_POSITION);
    }

    // Check if the motors have reached their target position within a tolerance
    public boolean isAtPosition(int targetPosition) {
        int currentPosition1 = armLift1.getCurrentPosition();
        int currentPosition2 = armLift2.getCurrentPosition();
        return Math.abs(currentPosition1 - targetPosition) <= POSITION_TOLERANCE &&
                Math.abs(currentPosition2 - targetPosition) <= POSITION_TOLERANCE;
    }

    // Check if both motors have reached their target positions exactly
    public boolean isAtTarget() {
        return !armLift1.isBusy() && !armLift2.isBusy();
    }

    // Get the current position of armLift1
    public int getCurrentPosition() {
        return armLift1.getCurrentPosition();
    }

    // Stop the motors
    public void stop() {
        armLift1.setPower(0);
        armLift2.setPower(0);
    }
}
