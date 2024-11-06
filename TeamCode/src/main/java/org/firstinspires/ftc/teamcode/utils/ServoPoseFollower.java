package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.List;

public class ServoPoseFollower {

    private final LiftUp liftUp;
    private final Arm arm;
    private final Wrist wrist;
    private final Claw claw;
    private List<ServoPose> poses;
    private final Timer poseTimer = new Timer();

    private int currentPoseIndex = 0;
    private boolean isComplete = false;

    public ServoPoseFollower(HardwareMap hardwareMap, List<ServoPose> poses) {
        this.liftUp = new LiftUp(hardwareMap);
        this.arm = new Arm(hardwareMap);
        this.wrist = new Wrist(hardwareMap);
        this.claw = new Claw(hardwareMap);
        this.poses = poses;
    }

    // Start the pose sequence from the beginning
    public void start() {
        poseTimer.resetTimer();
        currentPoseIndex = 0;
        isComplete = false;
        if (!poses.isEmpty()) {
            applyPose(poses.get(0));
        }
    }

    // Update the follower to handle timing and pose transitions
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

    // Set a new sequence of poses and restart
    public void setPoseSequence(List<ServoPose> newPoses) {
        this.poses = newPoses;
        start(); // Restart the follower with the new pose sequence
    }

    // Apply the servo positions for the given pose
    private void applyPose(ServoPose pose) {
        liftUp.setPosition(pose.getLiftUpPosition());
        arm.setPosition(pose.getArm0Position(), pose.getArm1Position());
        wrist.setPosition(pose.getWristPosition());
        claw.setPosition(pose.getClawPosition());
    }

    // Check if the current pose sequence is complete
    public boolean isComplete() {
        return isComplete;
    }
    // In ServoPoseFollower class
    public ServoPose getCurrentPose() {
        if (currentPoseIndex >= 0 && currentPoseIndex < poses.size()) {
            return poses.get(currentPoseIndex);
        }
        return null; // Return null if there is no valid current pose
    }

}
