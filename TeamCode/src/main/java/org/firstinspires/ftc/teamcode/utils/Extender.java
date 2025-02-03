package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Extender {
    private final DcMotor extender;
    private static final int zeroPose = 0;
    private static final int outPose = -2500;
    private static final int positionTolerance = 50;
    private static final double maxPower = 1.0;
    private static int targetPos = 0;

    public Extender(HardwareMap hardwareMap) {
       extender = hardwareMap.dcMotor.get("extendB");

       extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extender.setDirection(DcMotorSimple.Direction.REVERSE);

       extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setPosition(double position) {
        if(position > targetPos){
            while(extender.getCurrentPosition() < position){
                extender.setTargetPosition(targetPos);
                extender.setPower(maxPower);
                extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                targetPos -= 20;
            }
        }
        else{
            while(extender.getCurrentPosition() > position){
                extender.setTargetPosition(targetPos);
                extender.setPower(maxPower);
                extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                targetPos += 20;
            }
        }
    }

    public double getCurrentPosition(){
        return extender.getCurrentPosition();
    }

    public boolean isAtTarget(){
        return !extender.isBusy();
    }

    public boolean isAtPosition(int targetPosition) {
        int currentPosition = extender.getCurrentPosition();
        return Math.abs(currentPosition - targetPosition) <= positionTolerance;
    }

    public void stop(){
        extender.setPower(0);
    }
}