package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.List;

public class ServoPoseFollower {

    private final ArmB armB;
    private final Wrist wrist;
    private final Claw claw;
    private List<ServoPose> poses;
    private final Timer poseTimer = new Timer();

    private int currentPoseIndex = 0;
    private boolean isComplete = false;

    public ServoPoseFollower(HardwareMap hardwareMap, List<ServoPose> poses) {
        this.armB = new ArmB(hardwareMap);
        this.wrist = new Wrist(hardwareMap);
        this.claw = new Claw(hardwareMap);
        this.poses = poses;
    }

    public void start() {
        poseTimer.resetTimer();
        currentPoseIndex = 0;
        isComplete = false;
        if (!poses.isEmpty()) {
            applyPose(poses.get(0));
        }
    }


    public void update() {
        if (isComplete) return;

        if (poseTimer.getElapsedTime() >= poses.get(currentPoseIndex).getDuration()) {
            currentPoseIndex++;
            if (currentPoseIndex < poses.size()) {
                applyPose(poses.get(currentPoseIndex));
                poseTimer.resetTimer();
            } else {
                isComplete = true;
            }
        }
    }


    public void setPoseSequence(List<ServoPose> newPoses) {
        this.poses = newPoses;
        start();
    }


    private void applyPose(ServoPose pose) {
        claw.setPosition(pose.getClawBPosition(), pose.getClawTPosition());
        wrist.setPosition(pose.getWristBPosition(), pose.getWristTPosition());
        armB.setPosition(pose.getArmBPosition());

    }

    public boolean isComplete() {
        return isComplete;
    }
    public ServoPose getCurrentPose() {
        if (currentPoseIndex >= 0 && currentPoseIndex < poses.size()) {
            return poses.get(currentPoseIndex);
        }
        return null;
    }

}
