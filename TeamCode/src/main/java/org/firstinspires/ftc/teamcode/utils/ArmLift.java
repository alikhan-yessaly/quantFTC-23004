package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class ArmLift {
    private final DcMotor armLift1;
    private final DcMotor armLift2;
    private final TouchSensor touch;

    // Define positions and tolerance
    private static final int DOWN_POSITION = 0;
    private static final int UP_POSITION = 1300;
    private static final int POSITION_TOLERANCE = 50;  // Tolerance for "close enough" to target position
    private static final double MAX_POWER = 1.0;
    private static final int CLIP_UP_POSITION = 1300;
    public static final int CLIP_DOWN_POSITION = 0;
    private static int targetPos = 0;



    public ArmLift(HardwareMap hardwareMap) {
        armLift1 = hardwareMap.dcMotor.get("lift1");
        armLift2 = hardwareMap.dcMotor.get("lift2");
        touch = hardwareMap.touchSensor.get("touch");

        // Reset encoders and set modes
        armLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLift1.setDirection(DcMotor.Direction.REVERSE);

        armLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set the arm lift to a specified position
    public void setPosition(int position) {
        if (position > targetPos) {
            while (armLift1.getCurrentPosition() < position && armLift2.getCurrentPosition() < position) {
                armLift1.setTargetPosition(targetPos);
                armLift2.setTargetPosition(targetPos);
                armLift1.setPower(1);
                armLift2.setPower(1);
                armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                targetPos += 20;
            }
        }
//        else if(position <= targetPos){
//            while((armLift1.getCurrentPosition() > position) && armLift2.getCurrentPosition() > position) {
//                armLift1.setTargetPosition(targetPos);
//                armLift2.setTargetPosition(targetPos);
//                armLift1.setPower(1);
//                armLift2.setPower(1);
//                armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                if(!touch.isPressed()){
//                    targetPos -= 20;
//                }
//                else{
//                    break;
//                }
//
//            }
        else {
         while(armLift1.getCurrentPosition() > position && armLift2.getCurrentPosition() > position){
             armLift1.setTargetPosition(targetPos);
             armLift2.setTargetPosition(targetPos);
             armLift1.setPower(1);
             armLift2.setPower(1);
             armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             if(!touch.isPressed()){
                    targetPos -= 20;
                }
                else{
                    break;
                }

            }
        }
    }

    // Move the arm lift all the way down
    public void moveDown() {
       setPosition(DOWN_POSITION);
    }

    public void moveClipUp() {
        setPosition(CLIP_UP_POSITION);
    }

    public void moveClipDown() {
        setPosition(CLIP_DOWN_POSITION);
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
