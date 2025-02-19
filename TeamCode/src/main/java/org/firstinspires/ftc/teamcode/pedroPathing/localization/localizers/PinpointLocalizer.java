package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;


//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

import org.firstinspires.ftc.teamcode.pedroPathing.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.
//import com.pedropathing.localization.constants.PinpointConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.NanoTimer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Objects;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointLocalizer extends Localizer {
    private HardwareMap hardwareMap;
    private GoBildaPinpointDriver odo;
    private double previousHeading;
    private double totalHeading;
    private Pose startPose;
    private long deltaTimeNano;
    private NanoTimer timer;
    private Pose currentVelocity;
    private Pose pinpointPose;
    private boolean pinpointCooked;

    public PinpointLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public PinpointLocalizer(HardwareMap map, Pose setStartPose) {
        this.pinpointCooked = false;
        this.hardwareMap = map;
        this.odo = (GoBildaPinpointDriver)this.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        this.setOffsets(-2.25, -5.625, DistanceUnit.INCH);

        this.odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.resetPinpoint();
        this.setStartPose(setStartPose);
        this.totalHeading = 0.0;
        this.timer = new NanoTimer();
        this.pinpointPose = this.startPose;
        this.currentVelocity = new Pose();
        this.deltaTimeNano = 1L;
        this.previousHeading = setStartPose.getHeading();
    }

    public Pose getPose() {
        return this.pinpointPose.copy();
    }

    public Pose getVelocity() {
        return this.currentVelocity.copy();
    }

    public Vector getVelocityVector() {
        return this.currentVelocity.getVector();
    }

    public void setStartPose(Pose setStart) {
        if (!Objects.equals(this.startPose, new Pose()) && this.startPose != null) {
            Pose currentPose = MathFunctions.subtractPoses(MathFunctions.rotatePose(this.pinpointPose, -this.startPose.getHeading(), false), this.startPose);
            this.setPose(MathFunctions.addPoses(setStart, MathFunctions.rotatePose(currentPose, setStart.getHeading(), false)));
        } else {
            this.setPose(setStart);
        }

        this.startPose = setStart;
    }

    public void setPose(Pose setPose) {
        this.odo.setPosition(new Pose2D(DistanceUnit.INCH, setPose.getX(), setPose.getY(), AngleUnit.RADIANS, setPose.getHeading()));
        this.pinpointPose = setPose;
        this.previousHeading = setPose.getHeading();
    }

    public void update() {
        this.deltaTimeNano = this.timer.getElapsedTime();
        this.timer.resetTimer();
        this.odo.update();
        Pose currentPinpointPose = this.getPoseEstimate(this.odo.getPosition(), this.pinpointPose, this.deltaTimeNano);
        this.totalHeading += MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), this.previousHeading);
        this.previousHeading = currentPinpointPose.getHeading();
        Pose deltaPose = MathFunctions.subtractPoses(currentPinpointPose, this.pinpointPose);
        this.currentVelocity = new Pose(deltaPose.getX() / ((double)this.deltaTimeNano / Math.pow(10.0, 9.0)), deltaPose.getY() / ((double)this.deltaTimeNano / Math.pow(10.0, 9.0)), deltaPose.getHeading() / ((double)this.deltaTimeNano / Math.pow(10.0, 9.0)));
        this.pinpointPose = currentPinpointPose;
    }

    public double getTotalHeading() {
        return this.totalHeading;
    }

    public double getForwardMultiplier() {
        return (double)this.odo.getEncoderY();
    }

    public double getLateralMultiplier() {
        return (double)this.odo.getEncoderX();
    }

    public double getTurningMultiplier() {
        return (double)this.odo.getYawScalar();
    }

    private void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        this.odo.setOffsets(unit.toMm(xOffset), unit.toMm(yOffset));
    }

    public void resetIMU() throws InterruptedException {
        this.odo.recalibrateIMU();
    }

    private void resetPinpoint()  {
        this.odo.resetPosAndIMU();

        try {
            Thread.sleep(300L);
        } catch (InterruptedException var2) {
            InterruptedException e = var2;
            throw new RuntimeException(e);
        }
    }

    private Pose getPoseEstimate(Pose2D pinpointEstimate, Pose currentPose, long deltaTime) {
        if (!Double.isNaN(pinpointEstimate.getX(DistanceUnit.INCH)) && !Double.isNaN(pinpointEstimate.getY(DistanceUnit.INCH)) && !Double.isNaN(pinpointEstimate.getHeading(AngleUnit.RADIANS))) {
            Pose estimate = new Pose(pinpointEstimate.getX(DistanceUnit.INCH), pinpointEstimate.getY(DistanceUnit.INCH), pinpointEstimate.getHeading(AngleUnit.RADIANS));
            this.pinpointCooked = false;
            return estimate;
        } else {
            this.pinpointCooked = true;
            return MathFunctions.addPoses(currentPose, new Pose(this.currentVelocity.getX() * (double)deltaTime / Math.pow(10.0, 9.0), this.currentVelocity.getY() * (double)deltaTime / Math.pow(10.0, 9.0), this.currentVelocity.getHeading() * (double)deltaTime / Math.pow(10.0, 9.0)));
        }
    }

    public boolean isNAN() {
        return this.pinpointCooked;
    }
}