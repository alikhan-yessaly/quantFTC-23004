package org.firstinspires.ftc.teamcode.notcompetition.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.utils.ArmLift;
import org.firstinspires.ftc.teamcode.utils.ServoPose;
import org.firstinspires.ftc.teamcode.utils.ServoPoseFollower;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "testauto", group = "Autonomous")
public class testauto extends OpMode {
    private enum AutoState {
        INITIALIZE, FIRST_PATH, FIRST_POSE, SECOND_POSE, LIFT_UP, THIRD_POSE, PARK_PATH ,LIFT_DOWN, PARK_POSE, SECOND_PATH , THIRD_PATH , FOURTH_PATH , FIFTH_PATH , SIXTH_PATH , SEVENTH_PATH , COMPLETE
    }

    private AutoState currentState = AutoState.INITIALIZE;
    private Timer opmodeTimer = new Timer();
    private Follower follower;
    private ServoPoseFollower servoPoseFollower;
    private ArmLift armLift;

    private PathChain firstPath,secondPath, thirdPath , fourthPath , fifthPath , sixthPath , seventhPath;
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
        secondPath = buildSecondPath();
        thirdPath = buildThirdPath();
        fourthPath = buildFourthPath();
        fifthPath = buildFifthPath();
        sixthPath = buildSixthPath();
        seventhPath = buildSeventhPath();
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
                if (armLift.isAtTarget()) {
                setState(AutoState.THIRD_POSE);
                }
                break;
            case THIRD_POSE:
                if (servoPoseFollower.isComplete()) {
                    setState(AutoState.SECOND_PATH);
                }
                break;

            case SECOND_PATH:
                if (armLift.isAtPosition(0)) {
                    setState(AutoState.THIRD_PATH);
                }
                break;
            case THIRD_PATH:
                if (armLift.isAtPosition(0)) {
                    setState(AutoState.FOURTH_PATH);
                }
                break;
            case FOURTH_PATH:
                if (armLift.isAtPosition(0)) {
                    setState(AutoState.FIFTH_PATH);
                }
                break;
            case FIFTH_PATH:
                if (armLift.isAtPosition(0)) {
                    setState(AutoState.SIXTH_PATH);
                }
                break;
            case SIXTH_PATH:
                if (armLift.isAtPosition(0)) {
                    setState(AutoState.SEVENTH_PATH);
                }
                break;
            case SEVENTH_PATH:
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
            case SECOND_PATH:
                follower.followPath(secondPath);
                break;
            case THIRD_PATH:
                follower.followPath(thirdPath);
                break;
            case FOURTH_PATH:
                follower.followPath(fourthPath);
                break;
            case FIFTH_PATH:
                follower.followPath(fifthPath);
                break;
            case SIXTH_PATH:
                follower.followPath(sixthPath);
                break;
            case SEVENTH_PATH:
                follower.followPath(seventhPath);
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
                                new Point(40.500, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }
    private PathChain buildSecondPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(37.009, 71.551, Point.CARTESIAN),
                                new Point(20.411, 16.374, Point.CARTESIAN),
                                new Point(67.738, 46.206, Point.CARTESIAN),
                                new Point(61.907, 28.037, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    private PathChain buildThirdPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(61.907, 28.037, Point.CARTESIAN),
                                new Point(18.841, 27.589, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }
    private PathChain buildFourthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(18.841, 27.589, Point.CARTESIAN),
                                new Point(62.131, 28.037, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }

    private PathChain buildFifthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(62.131, 28.037, Point.CARTESIAN),
                                new Point(78.953, 13.682, Point.CARTESIAN),
                                new Point(17.495, 15.477, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }

    private PathChain buildSixthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(17.495, 15.477, Point.CARTESIAN),
                                new Point(70.000, 16.000, Point.CARTESIAN),
                                new Point(65.000, 10.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }

    private PathChain buildSeventhPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(65.000, 10.000, Point.CARTESIAN),
                                new Point(18.000, 10.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }

    // Define the initial servo poses
    private void defineInitialServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> initialPoses = Arrays.asList(
                new ServoPose(0.692, 0.789, 0.4995, 0.666, 0.35, 1000)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
    }

    // Define the second servo poses
    private void defineSecondServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> secondPoses = Arrays.asList(
                new ServoPose(0.692,0.4, 0.4995, 0.666, 0.35, 1000),
                new ServoPose(0.5105, 0, 0.228, 0.679, 0.65, 500)

        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, secondPoses);
    }
    // Define the second servo poses
    private void defineThirdServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> thirdPoses = Arrays.asList(
                new ServoPose(0.5105, 0, 0.228, 0.679, 0.35, 500),
                new ServoPose(0.7, 0, 0.228, 0.679, 0.35, 100)
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
