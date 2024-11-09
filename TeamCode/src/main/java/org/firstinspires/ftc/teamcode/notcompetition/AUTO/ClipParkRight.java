package org.firstinspires.ftc.teamcode.notcompetition.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.utils.ArmLift;
import org.firstinspires.ftc.teamcode.utils.ServoPose;
import org.firstinspires.ftc.teamcode.utils.ServoPoseFollower;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "ClipParkRight", group = "Autonomous")
public class ClipParkRight extends OpMode {
    private enum AutoState {
        INITIALIZE, FIRST_PATH, FIRST_POSE, SECOND_POSE, LIFT_UP, THIRD_POSE, LIFT_DOWN, PARK_POSE, COMPLETE
    }

    private AutoState currentState = AutoState.INITIALIZE;
    private Timer opmodeTimer = new Timer();
    private Follower follower;
    private ServoPoseFollower servoPoseFollower;
    private ArmLift armLift;

    private PathChain firstPath, parkPath;
    private static final Pose START_POSE = new Pose(144 - (63 + 72), 12 + 72, 0);

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE);

        armLift = new ArmLift(hardwareMap);
        armLift.moveDown();

        // Initialize servo poses for the initial state
        defineInitialServoPoses(hardwareMap);
        servoPoseFollower.start();

        telemetry.addData("Init Status", "Initialized with starting pose and servo positions.");
        telemetry.update();
    }

    @Override
    public void start() {
        firstPath = buildFirstPath();
        parkPath = buildParkPath();
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
                    setState(AutoState.SECOND_POSE);
                }
                break;

            case SECOND_POSE:
                if (servoPoseFollower.isComplete()) {
                    setState(AutoState.LIFT_UP);
                }
                break;
            case LIFT_UP:
//                if (armLift.isAtTarget()) {
                setState(AutoState.THIRD_POSE);
//                }
                break;
            case THIRD_POSE:
                if (servoPoseFollower.isComplete()) {
                    setState(AutoState.LIFT_DOWN);
                }
                break;
            case LIFT_DOWN:
                if (armLift.isAtPosition(0)) {
                    setState(AutoState.PARK_POSE);
                }
                break;
            case PARK_POSE:
                if (follower.isCloseEnoughToEnd()) {
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
            case SECOND_POSE:
                defineSecondServoPoses(hardwareMap); // Define the second servo pose
                servoPoseFollower.start();
                break;
            case LIFT_UP:
                armLift.moveUp();
                break;
            case THIRD_POSE:
                defineThirdServoPoses(hardwareMap);
                servoPoseFollower.start();
                break;
            case LIFT_DOWN:
                armLift.moveDown();
                defineFourthServoPoses(hardwareMap);
                servoPoseFollower.start();
                break;
            case PARK_POSE:
                follower.followPath(parkPath);
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
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 84.000, Point.CARTESIAN),
                                new Point(30.000, 84.000, Point.CARTESIAN),
                                new Point(17.000, 72.000, Point.CARTESIAN),
                                new Point(38.500, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
    private PathChain buildParkPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(38.500, 72.000, Point.CARTESIAN),
                                new Point(4.000, 76.000, Point.CARTESIAN),
                                new Point(14.000, 50.000, Point.CARTESIAN),
                                new Point(10.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    // Define the initial servo poses
    private void defineInitialServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> initialPoses = Arrays.asList(
                new ServoPose(0.0, 0.0, 0.2, 0.5, 0.65, 1000)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
    }

    // Define the second servo poses
    private void defineSecondServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> secondPoses = Arrays.asList(
                new ServoPose(0.2, 0.0, 0.2, 0.5, 0.65, 100),
                new ServoPose(0.4, 0.0, 0.2, 0.5, 0.65, 100),
                new ServoPose(0.6, 0.0, 0.2, 0.5, 0.65, 100),
                new ServoPose(0.7, 0.0, 0.2, 0.5, 0.65, 100)

        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, secondPoses);
    }
    // Define the second servo poses
    private void defineThirdServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> thirdPoses = Arrays.asList(
                new ServoPose(0.7, 0.0, 0.2, 0.5, 0.65, 500),
                new ServoPose(0.7, 0.0, 0.2, 0.5, 0.65, 100),
                new ServoPose(0.7, 0.2, 0.6, 0.5, 0.65, 100),
                new ServoPose(0.7, 0.4, 0.8, 0.5, 0.65, 100),
                new ServoPose(0.7, 0.6, 1.0, 0.5, 0.65, 100)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, thirdPoses);
    }
    private void defineFourthServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> fourthPoses = Arrays.asList(
                new ServoPose(0.7, 0.6, 1.0, 0.5, 0.65, 600),
                new ServoPose(0.7, 0.6, 1.0, 0.5, 0.2, 1000)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, fourthPoses);
    }
}