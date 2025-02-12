package org.firstinspires.ftc.teamcode.utils;

public class ServoPose {
    private final double clawT;
    private final double clawB;
    private final double wristB;
    private final double wristT;
    private final double armB;
    private final long duration;

    public ServoPose(double clawT, double clawB, double wristB, double wristT, double armB, long duration) {
        this.clawT = clawT;
        this.clawB = clawB;
        this.wristB = wristB;
        this.wristT = wristT;
        this.armB = armB;
        this.duration = duration;// Duration in milliseconds
    }

    public double getArmBPosition() { return armB; }
    public double getWristBPosition() { return wristB; }
    public double getWristTPosition() { return wristT; }
    public double getClawBPosition() { return clawB;  }
    public double getClawTPosition() { return clawT; }
    public long getDuration() { return duration; }
}

