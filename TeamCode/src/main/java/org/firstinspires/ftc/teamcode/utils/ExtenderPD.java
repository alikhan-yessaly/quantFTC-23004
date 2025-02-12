package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ExtenderPD {
    private Motor extenderMotor;
    private PDController extenderPD;
    private double extenderTargetPos;


    public ExtenderPD(HardwareMap hardwareMap){
        extenderMotor = new Motor(hardwareMap, "extender");
        extenderPD = new PDController(0, 0);
        extenderTargetPos = 0;

    }

    public void resetEncoder() { extenderMotor.resetEncoder(); }
    public void setTargetPosition(double position) { extenderTargetPos = position; }

    public void setPID(double kP, double kI, double kD){
        extenderPD.setPID(kP, kI, kD);
    }

    public void update(){
        double extenderPos = extenderMotor.getCurrentPosition();
        double extenderPower = extenderPD.calculate(extenderPos, extenderTargetPos);
        extenderMotor.set(extenderPower);
    }

    public double getCurrentPosition() { return extenderMotor.getCurrentPosition(); }

    public double getTargetPosition() { return  extenderTargetPos; }
}
