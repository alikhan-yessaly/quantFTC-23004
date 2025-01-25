package org.firstinspires.ftc.teamcode.utils;

public class ServoPose {
    private final double clawT;
    private final double clawB;
    private final double wristB;
    private final double wristT;
    private final double armB;
    private final double armT;
    private final double extender;
    private final long duration;

    public ServoPose(double clawT, double clawB, double wristB, double wristT, double armB, double armT, double extender, long duration) {
        this.clawB = clawB;
        this.clawT = clawT;
        this.wristB = wristB;
        this.wristT = wristT;
        this.armB = armB;
        this.armT = armT;
        this.extender = extender;
        this.duration = duration;// Duration in milliseconds
    }

    public double getArmBPosition() { return armB; }
    public double getArmTPosition() { return armT; }
    public double getWristBPosition() { return wristB; }
    public double getWristTPosition() { return wristT; }
    public double getClawBPosition() { return clawB;  }
    public double getClawTPosition() { return clawT; }
    public double getExtenderPosition() { return extender; }
    public long getDuration() { return duration; }
}

