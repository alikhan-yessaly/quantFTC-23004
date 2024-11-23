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

@Autonomous(name = "ClipParkFromRight", group = "Autonomous")
public class ClipParkFromRight extends OpMode {

    private enum AutoState {
        INITIALIZE, FIRST_PATH, FIRST_POSE, SECOND_POSE, LIFT_UP, THIRD_POSE, LIFT_DOWN,
        SECOND_PATH, THIRD_PATH, FOURTH_FORWARD, FOURTH_BACKWARD,
        FIFTH_PATH, SIXTH_FORWARD, SIXTH_BACKWARD,
        SEVENTH_PATH, EIGHTH_FORWARD, EIGHTH_BACKWARD, PARK_POSE, COMPLETE
    }

    private boolean delayDone = false;
    private AutoState currentState = AutoState.INITIALIZE;
    private Timer opmodeTimer = new Timer();
    private Follower follower;
    private ServoPoseFollower servoPoseFollower;
    private ArmLift armLift;

    private PathChain firstPath, parkPath, secondPath, thirdPath, fourthForwardPath, fourthBackwardPath,
            fifthPath, sixthForwardPath, sixthBackwardPath, seventhPath, eighthForwardPath, eighthBackwardPath;

    private static final Pose START_POSE = new Pose(9, 60, 0);

    @Override
    public void init() {
        try {
            // Инициализация подсистем
            follower = new Follower(hardwareMap);
            follower.setStartingPose(START_POSE);

            armLift = new ArmLift(hardwareMap);
            armLift.moveDown();

            // Инициализация сервоприводов
            defineInitialServoPoses(hardwareMap);
            servoPoseFollower.start();

            telemetry.addData("Init Status", "Initialization successful.");
        } catch (Exception e) {
            telemetry.addData("Error during init", e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void start() {
        try {
            // Инициализация путей
            firstPath = buildFirstPath();
            secondPath = buildSecondPath();
            thirdPath = buildThirdPath();
            fourthForwardPath = buildFourthForwardPath();
            fourthBackwardPath = buildFourthBackwardPath();
            fifthPath = buildFifthPath();
            sixthForwardPath = buildSixthForwardPath();
            sixthBackwardPath = buildSixthBackwardPath();
            seventhPath = buildSeventhPath();
            eighthForwardPath = buildEighthForwardPath();
            eighthBackwardPath = buildEighthBackwardPath();

            opmodeTimer.resetTimer();
            telemetry.addData("Path Status", "Paths loaded successfully.");
        } catch (Exception e) {
            telemetry.addData("Error during path load", e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        if (!delayDone && opmodeTimer.getElapsedTime() >= 0) {
            delayDone = true;
            setState(AutoState.FIRST_PATH);
        }

        follower.update();
        servoPoseFollower.update();

        switch (currentState) {
            case FIRST_PATH:
                if (follower.isCloseEnoughToEnd()) setState(AutoState.FIRST_POSE);
                break;
            case FIRST_POSE:
                if (servoPoseFollower.isComplete()) setState(AutoState.SECOND_POSE);
                break;
            case SECOND_POSE:
                if (servoPoseFollower.isComplete()) setState(AutoState.LIFT_UP);
                break;
            case LIFT_UP:
                setState(AutoState.THIRD_POSE);
                break;
            case THIRD_POSE:
                if (servoPoseFollower.isComplete()) setState(AutoState.LIFT_DOWN);
                break;
            case LIFT_DOWN:
                if (armLift.isAtPosition(0)) setState(AutoState.SECOND_PATH);
                break;
            case SECOND_PATH:
                if (follower.isCloseEnoughToEnd()) setState(AutoState.THIRD_PATH);
                break;
            case THIRD_PATH:
                if (follower.isCloseEnoughToEnd()) setState(AutoState.FOURTH_FORWARD);
                break;
            case FOURTH_FORWARD:
                if (follower.isCloseEnoughToEnd()) setState(AutoState.FOURTH_BACKWARD);
                break;
            case FOURTH_BACKWARD:
                if (follower.isCloseEnoughToEnd()) setState(AutoState.FIFTH_PATH);
                break;
            case FIFTH_PATH:
                if (follower.isCloseEnoughToEnd()) setState(AutoState.SIXTH_FORWARD);
                break;
            case SIXTH_FORWARD:
                if (follower.isCloseEnoughToEnd()) setState(AutoState.SIXTH_BACKWARD);
                break;
            case SIXTH_BACKWARD:
                if (follower.isCloseEnoughToEnd()) setState(AutoState.SEVENTH_PATH);
                break;
            case SEVENTH_PATH:
                if (follower.isCloseEnoughToEnd()) setState(AutoState.EIGHTH_FORWARD);
                break;
            case EIGHTH_FORWARD:
                if (follower.isCloseEnoughToEnd()) setState(AutoState.EIGHTH_BACKWARD);
                break;
            case EIGHTH_BACKWARD:
                if (follower.isCloseEnoughToEnd()) setState(AutoState.PARK_POSE);
                break;
            case PARK_POSE:
                if (follower.isCloseEnoughToEnd()) setState(AutoState.COMPLETE);
                break;
            case COMPLETE:
                telemetry.addData("Status", "Sequence Complete");
                break;
        }

        telemetry.addData("State", currentState);
        telemetry.addData("Current X", follower.getPose().getX());
        telemetry.addData("Current Y", follower.getPose().getY());
        telemetry.update();
    }

    private void setState(AutoState newState) {
        currentState = newState;
        opmodeTimer.resetTimer();
        try {
            switch (newState) {
                case FIRST_PATH:
                    follower.followPath(firstPath);
                    break;
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
                case PARK_POSE:
                    follower.followPath(parkPath);
                    break;
                case COMPLETE:
                    telemetry.addData("Status", "Autonomous Complete");
                    break;
            }
        } catch (Exception e) {
            telemetry.addData("Error setting state", e.getMessage());
        }
    }

private PathChain buildFirstPath() {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(9.0, 60.0, Point.CARTESIAN),
                                new Point(26.0, 58.0, Point.CARTESIAN),
                                new Point(38.5, 60.0, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    private PathChain buildSecondPath() {
        return follower.pathBuilder()
                .addPath(
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

    private PathChain buildThirdPath() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(71.103, 115.065, Point.CARTESIAN),
                                new Point(59.945, 120.426, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    private PathChain buildFourthForwardPath() {
        return follower.pathBuilder()
                .addPath(
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

    private PathChain buildFourthBackwardPath() {
        return follower.pathBuilder()
                .addPath(
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

    private PathChain buildFifthPath() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(61.966, 116.834, Point.CARTESIAN),
                                new Point(60.843, 124.916, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private PathChain buildSixthForwardPath() {
        return follower.pathBuilder()
                .addPath(
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

    private PathChain buildSixthBackwardPath() {
        return follower.pathBuilder()
                .addPath(
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

    private PathChain buildSeventhPath() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(61.067, 124.916, Point.CARTESIAN),
                                new Point(58.373, 132.101, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private PathChain buildEighthForwardPath() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(58.373, 132.101, Point.CARTESIAN),
                                new Point(19.982, 132.101, Point.CARTESIAN)
                        )
                )
                .setTangentialHeadingInterpolation()
                .setReversed(true)
                .build();
    }

    private PathChain buildEighthBackwardPath() {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(19.982, 132.101, Point.CARTESIAN),
                                new Point(58.822, 132.325, Point.CARTESIAN)
                        )
                )
                .setTangentialHeadingInterpolation()
                .build();
    }


    private void defineInitialServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> poses = Arrays.asList(
                new ServoPose(0.0, 0.0, 0.3, 0.5, 0.65, 1000)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, poses);
    }
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

