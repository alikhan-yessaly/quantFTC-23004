package org.firstinspires.ftc.teamcode.notcompetition.AUTO;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.utils.ArmTPD;
import org.firstinspires.ftc.teamcode.utils.ServoPose;
import org.firstinspires.ftc.teamcode.utils.ServoPoseFollower;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


@Autonomous(name = "Blue5Clip", group = "Autonomous")
public class BlueClip extends OpMode{

    private enum AutoState {
        FIRST_PATH,CLIP1,
        SECOND1_PATH,SECOND2_PATH,SECOND3_PATH,CLIP2_TAKE,THIRD_PATH, CLIP2,
        FOURTH1_PATH,FOURTH2_PATH,FOURTH3_PATH,FOURTH4_PATH,FOURTH5_PATH,CLIP3_TAKE,FIFTH_PATH, CLIP3,
        SIX1_PATH, SIX2_PATH, SIX3_PATH, CLIP4_TAKE, SEVENTH_PATH,CLIP4,
        EIGHT_PATH, CLIP5_TAKE, NINTH_PATH, CLIP5, PARK_PATH,
        COMPLETE;
    }
    private AutoState currentState = AutoState.FIRST_PATH;
    private Timer opmodeTimer = new Timer();
    private Follower follower;
    private ServoPoseFollower servoPoseFollower;
    private PathChain firstPath, second1Path,second2Path,second3Path,
            thirdPath,fourth1Path,fourth2Path,fourth3Path, fourth4Path,fourth5Path,fifthPath,
            six1Path, six2Path, six3Path, seventhPath, eightPath, ninePath, parkPath;

    private static final Pose START_POSE = new Pose(8.50, 58.00, Math.toRadians(180));

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE);


        defineInitialServoPoses(hardwareMap);
        servoPoseFollower.start();

        telemetry.addData("Init Status", "Initialized with starting pose and servo positions.");
        telemetry.update();

        firstPath = buildFirstPath();
        second1Path = buildSecond1Path();
        second2Path = buildSecond2Path();
        second3Path = buildSecond3Path();
        thirdPath = buildThirdPath();
        fourth1Path = buildFourth1Path();
        fourth2Path = buildFourth2Path();
        fourth3Path = buildFourth3Path();
        fourth4Path = buildFourth4Path();
        fourth5Path = buildFourth5Path();
        fifthPath = buildFifthPath();
        six1Path = build61Path();
        six2Path = build62Path();
        six3Path = build63Path();
        seventhPath = build7Path();
        eightPath = buildS8Path();
    }

    @Override
    public void start(){
        setState(AutoState.FIRST_PATH);
    }

    @Override
    public void loop(){
        follower.update();
        servoPoseFollower.update();
        switch (currentState){
            case FIRST_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.CLIP1);
                break;
            case CLIP1:
                if(servoPoseFollower.isComplete()) setState(AutoState.SECOND1_PATH);
                break;
            case SECOND1_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.SECOND2_PATH);
                break;
            case SECOND2_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.SECOND3_PATH);
                break;
            case SECOND3_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.CLIP2_TAKE);
                break;
            case CLIP2_TAKE:
                if(servoPoseFollower.isComplete()) setState(AutoState.THIRD_PATH);
                break;
            case THIRD_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.CLIP2);
                break;
            case CLIP2:
                if(servoPoseFollower.isComplete()) setState(AutoState.FOURTH1_PATH);
                break;
            case FOURTH1_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.FOURTH2_PATH);
                break;
            case FOURTH2_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.FOURTH3_PATH);
                break;
            case FOURTH3_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.CLIP3_TAKE);
                break;
            case CLIP3_TAKE:
                if(servoPoseFollower.isComplete()) setState(AutoState.FOURTH4_PATH);
                break;
            case FOURTH4_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.CLIP3);
                break;
            case CLIP3:
                if(servoPoseFollower.isComplete()) setState(AutoState.FOURTH5_PATH);
                break;
            case FOURTH5_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.FIFTH_PATH);
                break;
            case FIFTH_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.SIX1_PATH);
                break;
            case SIX1_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.CLIP4_TAKE);
                break;
            case CLIP4_TAKE:
                if(servoPoseFollower.isComplete()) setState(AutoState.SIX2_PATH);
                break;
            case SIX2_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.CLIP4);
                break;
            case CLIP4:
                if(servoPoseFollower.isComplete()) setState(AutoState.SIX3_PATH);
                break;
            case SIX3_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.CLIP5_TAKE);
                break;
            case CLIP5_TAKE:
                if(servoPoseFollower.isComplete()) setState(AutoState.SEVENTH_PATH);
                break;
            case SEVENTH_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.CLIP5);
                break;
            case CLIP5:
                if(servoPoseFollower.isComplete()) setState(AutoState.EIGHT_PATH);
                break;
            case EIGHT_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.COMPLETE);
                break;
            case COMPLETE:
                telemetry.addData("Status", "Autonomous Complete");
                break;
         }

        telemetry.addData("Current State", currentState);
        telemetry.addData("Current ServoPoseFollower State", servoPoseFollower.getCurrentPose().getArmTPosition());
        telemetry.addData("Follower X", follower.getPose().getX());
        telemetry.addData("Follower Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toRadians(follower.getPose().getHeading()));
//        telemetry.addData("Lifts", armLift.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop(){
        telemetry.addData("Status", "Autonomous Stopped");
        telemetry.update();
    }


    private void setState(AutoState newState){
        currentState = newState;
        opmodeTimer.resetTimer();
        switch (newState) {
            case FIRST_PATH: follower.followPath(firstPath); defineFirstServoPoses(hardwareMap);break;
            case CLIP1:  defineClipSetServoPoses(hardwareMap); servoPoseFollower.start(); break;
            case SECOND1_PATH: follower.followPath(second1Path);defineClipPrepareServoPoses(hardwareMap); break;
            case SECOND2_PATH: follower.followPath(second2Path); break;
            case SECOND3_PATH: follower.followPath(second3Path);break;
            case CLIP2_TAKE: defineClipTakeServoPoses(hardwareMap); break;
            case THIRD_PATH: follower.followPath(thirdPath); defineFirstServoPoses(hardwareMap);break;
            case CLIP2: defineClipSetServoPoses(hardwareMap); break;
            case FOURTH1_PATH: follower.followPath(fourth1Path); defineClipPrepareServoPoses(hardwareMap);break;
            case FOURTH2_PATH: follower.followPath(fourth2Path);defineClipPrepareServoPoses(hardwareMap);break;
            case FOURTH3_PATH: follower.followPath(fourth3Path);defineClipPrepareServoPoses(hardwareMap);break;
            case CLIP3_TAKE: defineClipTakeServoPoses(hardwareMap); break;
            case FOURTH4_PATH: follower.followPath(fourth4Path); defineFirstServoPoses(hardwareMap); break;
            case CLIP3: defineClipSetServoPoses(hardwareMap); break;
            case FOURTH5_PATH: follower.followPath(fourth5Path); defineClipPrepareServoPoses(hardwareMap);break;
            case FIFTH_PATH: follower.followPath(fifthPath);defineClipPrepareServoPoses(hardwareMap);break;
            case SIX1_PATH: follower.followPath(six1Path);defineClipPrepareServoPoses(hardwareMap);break;
            case CLIP4_TAKE: defineClipTakeServoPoses(hardwareMap); break;
            case SIX2_PATH: follower.followPath(six2Path); defineFirstServoPoses(hardwareMap);break;
            case CLIP4: defineClipSetServoPoses(hardwareMap); break;
            case SIX3_PATH: follower.followPath(six3Path);defineClipPrepareServoPoses(hardwareMap);break;
            case CLIP5_TAKE: defineClipTakeServoPoses(hardwareMap); break;
            case SEVENTH_PATH: follower.followPath(seventhPath); defineFirstServoPoses(hardwareMap);break;
            case CLIP5: defineClipSetServoPoses(hardwareMap); break;
            case EIGHT_PATH: follower.followPath(eightPath); break;
            case COMPLETE: stop(); break;

        }
    }

    private PathChain buildFirstPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(8.500, 58.000, Point.CARTESIAN),
                                new Point(39.200, 76.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private PathChain buildSecond1Path() {
        return follower.pathBuilder()
                .addPath(
                        // Line 2.1
                        new BezierCurve(
                                new Point(39.200, 76.000, Point.CARTESIAN),
                                new Point(23.486, 24.435, Point.CARTESIAN),
                                new Point(36.059, 33.687, Point.CARTESIAN),
                                new Point(66.900, 27.519, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private PathChain buildSecond2Path() {
        return follower.pathBuilder()
                .addPath(
                        // Line 2.2
                        new BezierLine(
                                new Point(66.900, 27.519, Point.CARTESIAN),
                                new Point(20.876, 19.928, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    private PathChain buildSecond3Path() {
        return follower.pathBuilder()
                .addPath(
                        // Line 2.3
                        new BezierLine(
                                new Point(20.876, 19.928, Point.CARTESIAN),
                                new Point(8.500, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private PathChain buildThirdPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(8.500, 24.000, Point.CARTESIAN),
                                new Point(26.916, 61.458, Point.CARTESIAN),
                                new Point(40.200, 70.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private PathChain buildFourth1Path() {
        return follower.pathBuilder()
                .addPath(
                        // Line 4.1
                        new BezierCurve(
                                new Point(40.200, 70.000, Point.CARTESIAN),
                                new Point(23.486, 24.435, Point.CARTESIAN),
                                new Point(36.059, 33.687, Point.CARTESIAN),
                                new Point(58.318, 15.477, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    private PathChain buildFourth2Path() {
        return follower.pathBuilder()
                .addPath(
                        // Line 4.2
                        new BezierLine(
                                new Point(58.318, 15.477, Point.CARTESIAN),
                                new Point(15.183, 16.369, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    private PathChain buildFourth3Path() {
        return follower.pathBuilder()
                .addPath(
                        // Line 4.3
                        new BezierCurve(
                                new Point(15.183, 16.369, Point.CARTESIAN),
                                new Point(23.103, 26.243, Point.CARTESIAN),
                                new Point(8.500, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }
    private PathChain buildFourth4Path(){
        return follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(8.500, 24.000, Point.CARTESIAN),
                                new Point(26.916, 61.458, Point.CARTESIAN),
                                new Point(40.200, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    private PathChain buildFourth5Path(){
        return follower.pathBuilder()
                .addPath(
                        // Line 6.1
                        new BezierCurve(
                                new Point(40.200, 72.000, Point.CARTESIAN),
                                new Point(26.000, 42.000, Point.CARTESIAN),
                                new Point(36.059, 33.687, Point.CARTESIAN),
                                new Point(61.918, 8.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    private PathChain buildFifthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 6.2
                        new BezierLine(
                                new Point(61.918, 8.500, Point.CARTESIAN),
                                new Point(18.165, 10.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }


    private PathChain build61Path() {
        return follower.pathBuilder()
                .addPath(
                        // Line 6.3
                        new BezierLine(
                                new Point(18.165, 10.000, Point.CARTESIAN),
                                new Point(8.500, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private PathChain build62Path() {
        return follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(8.500, 24.000, Point.CARTESIAN),
                                new Point(26.916, 61.458, Point.CARTESIAN),
                                new Point(40.200, 71.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    private PathChain build63Path() {
        return follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(40.200, 71.000, Point.CARTESIAN),
                                new Point(8.500, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private PathChain build7Path() {
        return follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(8.500, 24.000, Point.CARTESIAN),
                                new Point(26.916, 61.458, Point.CARTESIAN),
                                new Point(40.200, 70.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private PathChain buildS8Path() {
        return follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(40.200, 70.000, Point.CARTESIAN),
                                new Point(8.500, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

//    private PathChain buildS9Path() {
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 9
//                        new BezierLine(
//                                new Point(8.00, 26.000, Point.CARTESIAN),
//                                new Point(40.200, 70.000, Point.CARTESIAN)
//                        )
//                ).setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//    }
//
//
//    private PathChain ParkPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 8
//                        new BezierLine(
//                                new Point(40.200, 70.000, Point.CARTESIAN),
//                                new Point(8.500, 24.000, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
//                .build();
//    }

    private void defineClipSetServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> initialPoses = Arrays.asList(
                new ServoPose(0.65, 0.35, 0.55, 0.4, 1,2000,300),
                new ServoPose(0.35, 0.35, 0.55, 0.4, 1,2000,100)

        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
    }

    private void defineClipPrepareServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> initialPoses = Arrays.asList(
                new ServoPose(0.35, 0.35, 0.55, 0.6, 1,0,300)

        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
    }

    private void defineClipTakeServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> initialPoses = Arrays.asList(
                new ServoPose(0.35, 0.35, 0.55, 0.6, 1,0,300),
                new ServoPose(0.35, 0.35, 0.55, 0.6, 1,450,300),
                new ServoPose(0.7, 0.35, 0.55, 0.6, 1,450,200)

        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
    }


    private void defineInitialServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> initialPoses = Arrays.asList(
                new ServoPose(0.65, 0.35, 0.55, 1.0, 1,0,100)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
    }


    private void defineFirstServoPoses(HardwareMap hardwareMap){
        List<ServoPose> secondPoses = Arrays.asList(
                new ServoPose(0.65, 0.35, 0.55, 1.0, 1,1000,100),
                new ServoPose(0.65, 0.35, 0.55, 0.8, 1,2000,100),
                new ServoPose(0.65, 0.35, 0.55, 0.6, 1,3000,100),
                new ServoPose(0.65, 0.35, 0.55, 0.5, 1,4000,100)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, secondPoses);
    }



}
