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
import org.firstinspires.ftc.teamcode.utils.ServoPose;
import org.firstinspires.ftc.teamcode.utils.ServoPoseFollower;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "AutoPathing", group = "Autonomous")
public class AutoPathing extends OpMode {

    private enum AutoState {
        FIRST_PATH, SECOND_PATH, THIRD_PATH, FOURTH_FORWARD, FOURTH_BACKWARD, FIFTH_PATH, SIXTH_FORWARD, SIXTH_BACKWARD, SEVENTH_PATH, EIGHTH_FORWARD, EIGHTH_BACKWARD, COMPLETE
    }

    private AutoState currentState = AutoState.FIRST_PATH;
    private Follower follower;
    private PathChain firstPath, secondPath, thirdPath, fourthForwardPath, fourthBackwardPath, fifthPath, sixthForwardPath, sixthBackwardPath, seventhPath, eighthForwardPath, eighthBackwardPath;
    private ServoPoseFollower servoPoseFollower;
    private static final Pose START_POSE = new Pose(144 - (63 + 72), 12 + 72, 0);

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE);
        telemetry.addData("Init Status", "Initialized with starting pose.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Generate paths
        firstPath = FirstPath();
        secondPath = SecondPath();
        thirdPath = ThirdPath();
        fourthForwardPath = FourthForwardPath();
        fourthBackwardPath = FourthBackwardsPath();
        fifthPath = FifthPath();
        sixthForwardPath = SixthForwardPath();
        sixthBackwardPath = SixthBackwardPath();
        seventhPath = SeventhPath();
        eighthForwardPath = EighthForwardPath();
        eighthBackwardPath = EighthBackwardPath();


        // Start with the first path
        follower.followPath(firstPath);
    }

    @Override
    public void loop() {
        follower.update();
        switch (currentState) {
            case FIRST_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.SECOND_PATH);
                }
                break;
            case SECOND_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.THIRD_PATH);
                }
                break;
            case THIRD_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.FOURTH_FORWARD);
                }
                break;
            case FOURTH_FORWARD:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.FOURTH_BACKWARD);
                }
                break;
            case FOURTH_BACKWARD:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.FIFTH_PATH);
                }
                break;
            case FIFTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.SIXTH_FORWARD);
                }
                break;
            case SIXTH_FORWARD:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.SIXTH_BACKWARD);
                }
                break;
            case SIXTH_BACKWARD:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.SEVENTH_PATH);
                }
                break;
            case SEVENTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.EIGHTH_FORWARD);
                }
                break;
            case EIGHTH_FORWARD:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.EIGHTH_BACKWARD);
                }
                break;
            case EIGHTH_BACKWARD:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.COMPLETE);
                }
                break;
            case COMPLETE:
                telemetry.addData("Status", "Autonomous Complete");
                break;
        }

        telemetry.addData("Current State", currentState);
        telemetry.addData("Follower X", follower.getPose().getX());
        telemetry.addData("Follower Y", follower.getPose().getY());
        telemetry.update();
    }

    private void setState(AutoState newState) {
        currentState = newState;
        switch (newState) {
            case SECOND_PATH:
                follower.followPath(secondPath);
                break;
            case THIRD_PATH:
                follower.followPath(thirdPath);
                break;
            case FOURTH_FORWARD:
                follower.followPath(fourthForwardPath);
                break;
            case FOURTH_BACKWARD:
                follower.followPath(fourthBackwardPath);
                break;
            case FIFTH_PATH:
                follower.followPath(fifthPath);
                break;
            case SIXTH_FORWARD:
                follower.followPath(sixthForwardPath);
                break;
            case SIXTH_BACKWARD:
                follower.followPath(sixthBackwardPath);
                break;
            case SEVENTH_PATH:
                follower.followPath(seventhPath);
                break;
            case EIGHTH_FORWARD:
                follower.followPath(eighthForwardPath);
                break;
            case EIGHTH_BACKWARD:
                follower.followPath(eighthBackwardPath);
                break;
            case COMPLETE:
                telemetry.addData("Status", "Sequence Complete");
                break;
        }
    }
    @Override
    public void stop() {
        telemetry.addData("Status", "Autonomous Stopped");
        telemetry.update();
    }


//    private PathChain FirstPath() {
//        return follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                        // Line 1
//                        new Point(7.096, 84.313, Point.CARTESIAN),
//                        new Point(26.296, 85.148, Point.CARTESIAN),
//                        new Point(16.838, 70.809, Point.CARTESIAN),
//                        new Point(38.000, 72.000, Point.CARTESIAN)
//
//                        ))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//        .build();
//    }
    private PathChain FirstPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.757, 84.983, Point.CARTESIAN),
                                new Point(21.287, 85.148, Point.CARTESIAN),
                                new Point(18.157, 70.748, Point.CARTESIAN),
                                new Point(38.500, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
    private void defineInitialServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> initialPoses = Arrays.asList(
                new ServoPose(0.0, 0.0, 0.3, 0.5, 0.65, 1000)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
    }
    private PathChain SecondPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(38.500, 72.000, Point.CARTESIAN),
                                new Point(9.205, 129.407, Point.CARTESIAN),
                                new Point(42.208, 119.528, Point.CARTESIAN),
                                new Point(71.103, 115.065, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
    private PathChain ThirdPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(71.103, 115.065, Point.CARTESIAN),
                                new Point(59.945, 120.426, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
    private PathChain FourthForwardPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(61.966, 116.609, Point.CARTESIAN),
                                new Point(29.636, 120.426, Point.CARTESIAN),
                                new Point(23.776, 114.168, Point.CARTESIAN),
                                new Point(13.695, 119.528, Point.CARTESIAN)
                        )
                )
                .setTangentialHeadingInterpolation()
                .setReversed(true)
                .build();
    }
    private PathChain FourthBackwardsPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(13.695, 119.528, Point.CARTESIAN),
                                new Point(23.776, 114.168, Point.CARTESIAN),
                                new Point(29.636, 120.426, Point.CARTESIAN),
                                new Point(61.966, 116.834, Point.CARTESIAN)
                        )
                )
                .setTangentialHeadingInterpolation()

                .build();
    }
    private PathChain FifthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(61.966, 116.834, Point.CARTESIAN),
                                new Point(60.843, 124.916, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    private PathChain SixthForwardPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(60.843, 124.916, Point.CARTESIAN),
                                new Point(31.207, 135.019, Point.CARTESIAN),
                                new Point(23.798, 126.263, Point.CARTESIAN),
                                new Point(17.961, 127.162, Point.CARTESIAN)
                        )
                )
                .setTangentialHeadingInterpolation()
                .setReversed(true)
                .build();
    }
    private PathChain SixthBackwardPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(17.961, 127.162, Point.CARTESIAN),
                                new Point(22.227, 125.814, Point.CARTESIAN),
                                new Point(31.207, 135.019, Point.CARTESIAN),
                                new Point(61.067, 124.916, Point.CARTESIAN)
                        )
                )
                .setTangentialHeadingInterpolation()

                .build();
    }
    private PathChain SeventhPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(61.067, 124.916, Point.CARTESIAN),
                                new Point(58.373, 132.101, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    private PathChain EighthForwardPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(58.373, 132.101, Point.CARTESIAN),
                                new Point(19.982, 132.101, Point.CARTESIAN)
                        )
                )
                .setTangentialHeadingInterpolation()
                .setReversed(true)
                .build();
    }
    private PathChain EighthBackwardPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(19.982, 132.101, Point.CARTESIAN),
                                new Point(58.822, 132.325, Point.CARTESIAN)
                        )
                )
                .setTangentialHeadingInterpolation()

                .build();
    }


}


