package org.firstinspires.ftc.teamcode.utils;

public class ServoPose {
    private final double liftUpPosition;
    private final double arm0Position;
    private final double arm1Position;
    private final double wristPosition;
    private final double clawPosition;
    private final long duration;

    public ServoPose(double liftUpPosition, double arm0Position, double arm1Position, double wristPosition, double clawPosition, long duration) {
        this.liftUpPosition = liftUpPosition;
        this.arm0Position = arm0Position;
        this.arm1Position = arm1Position;
        this.wristPosition = wristPosition;
        this.clawPosition = clawPosition;
        this.duration = duration; // Duration in milliseconds
    }

    public double getLiftUpPosition() { return liftUpPosition; }
    public double getArm0Position() { return arm0Position; }
    public double getArm1Position() { return arm1Position; }
    public double getWristPosition() { return wristPosition; }
    public double getClawPosition() { return clawPosition; }
    public long getDuration() { return duration; }
}

