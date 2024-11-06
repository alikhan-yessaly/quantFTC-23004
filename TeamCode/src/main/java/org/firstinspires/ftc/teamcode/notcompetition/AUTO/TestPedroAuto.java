package org.firstinspires.ftc.teamcode.notcompetition.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.ArmLift;
import org.firstinspires.ftc.teamcode.utils.ServoPose;
import org.firstinspires.ftc.teamcode.utils.ServoPoseFollower;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "TestPedroAuto", group = "Autonomous")
public class TestPedroAuto extends OpMode {

    private enum AutoState {
        INITIALIZE, FIRST_PATH, FIRST_POSE, LIFT_DOWN, SECOND_POSE, COMPLETE
    }

    private AutoState currentState = AutoState.INITIALIZE;
    private Timer opmodeTimer = new Timer();
    private Follower follower;
    private ServoPoseFollower servoPoseFollower;
    private ArmLift armLift;

    private PathChain firstPath;
    private static final Pose START_POSE = new Pose(144 - (63 + 72), 12 + 72, 0);

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE);

        armLift = new ArmLift(hardwareMap);

        // Initialize servo poses for the initial state
        defineInitialServoPoses(hardwareMap);

        telemetry.addData("Init Status", "Initialized with starting pose and servo positions.");
        telemetry.update();
    }

    @Override
    public void start() {
        firstPath = buildFirstPath();
        setState(AutoState.FIRST_PATH);
    }

    @Override
    public void loop() {
        follower.update();
        servoPoseFollower.update();

        switch (currentState) {
            case FIRST_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.FIRST_POSE);
                }
                break;
            case FIRST_POSE:
                if (servoPoseFollower.isComplete()) {
                    setState(AutoState.LIFT_DOWN);
                }
                break;
            case LIFT_DOWN:
                if (armLift.isAtPosition(0)) {
                    setState(AutoState.SECOND_POSE);
                }
                break;
            case SECOND_POSE:
                if (servoPoseFollower.isComplete()) {
                    setState(AutoState.COMPLETE);
                }
                break;
            case COMPLETE:
                telemetry.addData("Status", "Sequence Complete");
                break;
        }

        telemetry.addData("Current State", currentState);
        telemetry.addData("Follower X", follower.getPose().getX());
        telemetry.addData("Follower Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Autonomous Stopped");
        telemetry.update();
    }

    private void setState(AutoState newState) {
        currentState = newState;
        opmodeTimer.resetTimer();
        switch (newState) {
            case FIRST_PATH:
                follower.followPath(firstPath);
                break;
            case FIRST_POSE:
                defineInitialServoPoses(hardwareMap); // Define the initial servo pose
                servoPoseFollower.start();
                break;
            case LIFT_DOWN:
                armLift.moveDown();
                break;
            case SECOND_POSE:
                defineSecondServoPoses(hardwareMap); // Define the second servo pose
                servoPoseFollower.start();
                break;
            case COMPLETE:
                telemetry.addData("Status", "Autonomous Complete");
                break;
            default:
                break;
        }
    }

    private PathChain buildFirstPath() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(7.182, 90.359, Point.CARTESIAN),
                        new Point(33.890, 92.603, Point.CARTESIAN),
                        new Point(16.833, 74.873, Point.CARTESIAN),
                        new Point(39.277, 75.546, Point.CARTESIAN)
                ))
                .setTangentHeadingInterpolation()
                .build();
    }

    // Define the initial servo poses
    private void defineInitialServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> initialPoses = Arrays.asList(
                new ServoPose(0, 0, 0.2, 0.5, 0.4, 1000)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
    }

    // Define the second servo poses
    private void defineSecondServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> secondPoses = Arrays.asList(
                new ServoPose(1.0, 0.0, 0.7, 0.5, 0.3, 1200)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, secondPoses);
    }
}
