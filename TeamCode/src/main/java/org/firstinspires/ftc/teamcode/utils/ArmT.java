package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmT {
    private final DcMotor armT;

    private static final int zeroPose = 0;
    private static final int outPose = 8000;
    private static final int positionTolerance = 50;
    private static final double maxPower = 1.0;
    private static int targetPos = 0;

    public ArmT(HardwareMap hardwareMap){
        armT = hardwareMap.dcMotor.get("armT");

        armT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPosition(int position){
        targetPos = position; // Store the desired target position
    }

    public void update() {
        int currentPos = armT.getCurrentPosition();

        // Smooth approach
        if (Math.abs(currentPos - targetPos) > positionTolerance) {
            if (currentPos < targetPos) {
                currentPos += 20; // Step up smoothly
            } else {
                currentPos -= 20; // Step down smoothly
            }
            armT.setTargetPosition(currentPos);
            armT.setPower(maxPower);
            armT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public int getCurrentPosition() {
        return armT.getCurrentPosition();
    }

    // Check if both motors have reached their target positions exactly
    public boolean isAtTarget(){
        return !armT.isBusy();
    }

    public boolean isAtPosition(int targetPosition) {
        int currentPosition = armT.getCurrentPosition();
        return Math.abs(currentPosition - targetPosition) <= positionTolerance;
    }

    public void stop(){
        armT.setPower(0);
    }

}
