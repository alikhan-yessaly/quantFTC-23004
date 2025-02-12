package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftPD {
    private Motor lift1;
    private Motor lift2;
    private PDController lift1PD;
    private PDController lift2PD;
    private double liftTargetPos;

    public LiftPD(HardwareMap hardwareMap){
        lift1 = new Motor(hardwareMap, "lift1");
        lift2 = new Motor(hardwareMap, "lift2");

        lift2.setInverted(true);

        lift1PD = new PDController(0,0);
        lift2PD = new PDController(0,0);

        liftTargetPos = 0;

        lift1.resetEncoder();
        lift2.resetEncoder();
    }

    public void setTargetPosition(double position){
        liftTargetPos = position;
    }

    public void setPID(double kP, double kI, double kD){
        lift1PD.setPID(kP, kI, kD);
        lift2PD.setPID(kP, kI, kD);
    }

    public void update(){
        double lift1Pos = lift1.getCurrentPosition();
        double lift2Pos = lift2.getCurrentPosition();

        double lift1Power = lift1PD.calculate(lift1Pos, liftTargetPos);
        double lift2Power = lift2PD.calculate(lift2Pos, liftTargetPos);

        lift1.set(lift1Power);
        lift2.set(lift2Power);
    }

    public double getLiftLCurrentPosition(){ return lift1.getCurrentPosition(); }
    public double getLiftRCurrentPosition(){ return lift2.getCurrentPosition(); }

    public double getTargetPosition() { return liftTargetPos; }
}
