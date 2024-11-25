package org.firstinspires.ftc.teamcode.notcompetition.AUTO;

import
        com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "ClipParkRight", group = "Autonomous")
public class ClipParkRight extends OpMode {
    private enum AutoState {
        INITIALIZE, FIRST_PATH , FIRST_POSE, SECOND_POSE , THIRD_PATH  , FOURTH_PATH , FIFTH_PATH , SIXTH_PATH , SEVENTH_PATH , EIGHTH_PATH , NINTH_PATH, TENTH_PATH , ELEVENTH_PATH , TWELETH_PATH , THIRTEENTH_PATH , FOURTEENTH_PATH , FIFTEENTH_PATH , SIXTEENTH_PATH , SEVENTEENTH_PATH, SECOND_PATH, COMPLETE
    }

    private AutoState currentState = AutoState.INITIALIZE;
    private Timer opmodeTimer = new Timer();
    private Follower follower;
    private ServoPoseFollower servoPoseFollower;
    private ArmLift armLift;

    private PathChain firstPath, secondPath , thirdPath , fourthPath , fifthPath , sixthPath , seventhPath , eighthPath , ninthPath , tenthPath , eleventhPath , twelfthPath , thirteenthPath , fourteenthPath , fifteenthPath , sixteenthPath , seventeenthPath;
    private static final Pose START_POSE = new Pose(11.215, 60.336, 180);

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE);

//        Initialize servo poses for the initial state
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
        eighthPath = buildEighthPath();
        ninthPath = buildNinthPath();
        tenthPath = buildTenthPath();
        eleventhPath = buildEleventhPath();
        twelfthPath = buildTwelthPath();
        thirteenthPath = buildThirteenthPath();
        fourteenthPath = buildFourteenthPath();
        fifteenthPath = buildFifteenthPath();
        sixteenthPath = buildSixteenthPath();
        seventeenthPath = buildSeventeenthPath();
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
//            case WAIT_AFTER_FIRST_PATH:
//                // Ждем 1.5 секунды перед началом второго пути
//                if (opmodeTimer.getElapsedTime() >= 0.5) {
//                    setState(AutoState.FIRST_POSE);
//                }
//                break;
            case FIRST_POSE:
                if (servoPoseFollower.isComplete()) {
                    setState(AutoState.SECOND_POSE);
                }
                break;

            case SECOND_POSE:
                if (servoPoseFollower.isComplete()) {
                    setState(AutoState.SECOND_PATH);
                }
                break;
//            case LIFT_UP:
//                if (armLift.isAtTarget()) {
//                setState(AutoState.THIRD_POSE);
//                }
//                break;
//            case THIRD_POSE:
//                if (servoPoseFollower.isComplete()) {
//                    setState(AutoState.LIFT_DOWN);
//                }
//                break;
//            case LIFT_DOWN:
//                if (armLift.isAtPosition(0)) {
//                    setState(AutoState.SECOND_PATH);
//                }
//                break;
            case SECOND_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.THIRD_PATH);
                }
            case THIRD_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.FOURTH_PATH);
                }
            case FOURTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.FIFTH_PATH);
                }
                break;
            case FIFTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.SIXTH_PATH);
                }
                break;
            case SIXTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.SEVENTH_PATH);
                }
                break;
            case SEVENTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.EIGHTH_PATH);
                }
                break;
            case EIGHTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.NINTH_PATH);
                }
                break;
            case NINTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.TENTH_PATH);
                }
            case TENTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.ELEVENTH_PATH);
                }
                break;
            case ELEVENTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.TWELETH_PATH);
                }
            case TWELETH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.THIRTEENTH_PATH);
                }
                break;
            case THIRTEENTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.FOURTEENTH_PATH);
                }
                break;
            case FOURTEENTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.FIFTEENTH_PATH);
                }
                break;
            case FIFTEENTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.SIXTEENTH_PATH);
                }
                break;
            case SIXTEENTH_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setState(AutoState.SEVENTEENTH_PATH);
                }
                break;
            case SEVENTEENTH_PATH:
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
//        opmodeTimer.resetTimer();
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
//            case LIFT_UP:
//                armLift.moveUp();
//                break;
//            case THIRD_POSE:
//                defineThirdServoPoses(hardwareMap);
//                servoPoseFollower.start();
//                break;
//            case LIFT_DOWN:
//                armLift.moveDown();
//                defineFourthServoPoses(hardwareMap);
//                servoPoseFollower.start();
//                break;
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
            case EIGHTH_PATH:
                follower.followPath(eighthPath);
                break;
            case NINTH_PATH:
                follower.followPath(ninthPath);
                break;
            case TENTH_PATH:
                follower.followPath(tenthPath);
                break;
            case ELEVENTH_PATH:
                follower.followPath(eleventhPath);
                break;
            case TWELETH_PATH:
                follower.followPath(twelfthPath);
                break;
            case THIRTEENTH_PATH:
                follower.followPath(thirteenthPath);
                break;
            case FOURTEENTH_PATH:
                follower.followPath(fourteenthPath);
                break;
            case FIFTEENTH_PATH:
                follower.followPath(fifteenthPath);
                break;
            case SIXTEENTH_PATH:
                follower.followPath(sixteenthPath);
                break;
            case SEVENTEENTH_PATH:
                follower.followPath(seventeenthPath);
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
                                new Point(11.215, 60.336, Point.CARTESIAN),
                                new Point(26.916, 58.318, Point.CARTESIAN),
                                new Point(25.794, 74.692, Point.CARTESIAN),
                                new Point(37.009, 71.551, Point.CARTESIAN)
                        )
                )
                .setTangentialHeadingInterpolation()
                .setReversed(true)
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
    private PathChain buildEighthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(18.000, 10.000, Point.CARTESIAN),
                                new Point(27.140, 34.542, Point.CARTESIAN),
                                new Point(61.907, 28.037, Point.CARTESIAN)
                        )
                )

                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }
    private PathChain buildNinthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(61.907, 28.037, Point.CARTESIAN),
                                new Point(38.804, 28.262, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }
    private PathChain buildTenthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(38.804, 28.262, Point.CARTESIAN),
                                new Point(10.093, 28.262, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }
    private PathChain buildEleventhPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(10.093, 28.262, Point.CARTESIAN),
                                new Point(11.215, 71.776, Point.CARTESIAN),
                                new Point(37.682, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }
    private PathChain buildTwelthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(37.682, 72.000, Point.CARTESIAN),
                                new Point(28.262, 28.486, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }
    private PathChain buildThirteenthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(28.262, 28.486, Point.CARTESIAN),
                                new Point(10.093, 28.486, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }
    private PathChain buildFourteenthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 14
                        new BezierCurve(
                                new Point(10.093, 28.486, Point.CARTESIAN),
                                new Point(10.542, 71.103, Point.CARTESIAN),
                                new Point(37.458, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }
    private PathChain buildFifteenthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 15
                        new BezierLine(
                                new Point(37.458, 72.000, Point.CARTESIAN),
                                new Point(28.262, 28.262, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }
    private PathChain buildSixteenthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 16
                        new BezierLine(
                                new Point(28.262, 28.262, Point.CARTESIAN),
                                new Point(10.093, 28.486, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }
    private PathChain buildSeventeenthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 17
                        new BezierCurve(
                                new Point(10.093, 28.486, Point.CARTESIAN),
                                new Point(11.888, 72.000, Point.CARTESIAN),
                                new Point(37.458, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }




//     Define the initial servo poses
    private void defineInitialServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> initialPoses = Arrays.asList(
                new ServoPose(0.692, 0.789, 0.4995, 0.666, 0.35, 1000)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
    }

//     Define the second servo poses
    private void defineSecondServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> secondPoses = Arrays.asList(
                new ServoPose(0.692,0.4, 0.4995, 0.666, 0.35, 1000),
                new ServoPose(0.5105, 0, 0.228, 0.679, 0.65, 500)

        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, secondPoses);
    }
    // Define the third servo poses
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