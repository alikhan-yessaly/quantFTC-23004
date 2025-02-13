package org.firstinspires.ftc.teamcode.notcompetition.AUTO;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
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


@Autonomous(name = "BlueClip", group = "Autonomous")
public class BlueClip extends OpMode{

    private enum AutoState {
        FIRST_PATH,CLIP1,SECOND_PATH,CLIP2_TAKE,THIRD_PATH,CLIP2,FOURTH_PATH,CLIP3_TAKE,FIFTH_PATH, CLIP3, COMPLETE;
    }
    private AutoState currentState = AutoState.FIRST_PATH;
    private Timer opmodeTimer = new Timer();
    private Follower follower;
    private ServoPoseFollower servoPoseFollower;
    private PathChain firstPath, secondPath, thirdPath,fourthPath, fifthPath, firstPose, secondPose, liftUp, thirdPose, liftDown, parkPath;

    private static final Pose START_POSE = new Pose(9.00, 58.00, Math.toRadians(180));

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE);


        defineInitialServoPoses(hardwareMap);
        servoPoseFollower.start();

        telemetry.addData("Init Status", "Initialized with starting pose and servo positions.");
        telemetry.update();

        firstPath = buildFirstPath();
        secondPath = buildSecondPath();
        thirdPath = buildThirdPath();
        fourthPath = buildFourthPath();
        fifthPath = buildFifthPath();
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
                if(servoPoseFollower.isComplete()) setState(AutoState.SECOND_PATH);
                break;
            case SECOND_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.CLIP2_TAKE);
                break;
            case CLIP2_TAKE:
                if(servoPoseFollower.isComplete()) setState(AutoState.THIRD_PATH);
                break;
            case THIRD_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.CLIP2);
                break;
            case CLIP2:
                if(servoPoseFollower.isComplete()) setState(AutoState.FOURTH_PATH);
                break;
            case FOURTH_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.CLIP3_TAKE);
                break;
            case CLIP3_TAKE:
                if(servoPoseFollower.isComplete()) setState(AutoState.FIFTH_PATH);
                break;
            case FIFTH_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.CLIP3);
                break;
            case CLIP3:
                if(servoPoseFollower.isComplete()) setState(AutoState.COMPLETE);
                break;

            case COMPLETE:
                telemetry.addData("Status", "Autonomous Complete");
                break;
         }

        telemetry.addData("Current State", currentState);
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
            case SECOND_PATH: follower.followPath(secondPath);defineClipPrepareServoPoses(hardwareMap); break;
            case CLIP2_TAKE: defineClipTakeServoPoses(hardwareMap); break;
            case THIRD_PATH: follower.followPath(thirdPath); defineFirstServoPoses(hardwareMap);break;
            case CLIP2: defineClipSetServoPoses(hardwareMap); break;
            case FOURTH_PATH: follower.followPath(fourthPath); defineClipPrepareServoPoses(hardwareMap);break;
            case CLIP3_TAKE: defineClipTakeServoPoses(hardwareMap); break;
            case FIFTH_PATH: follower.followPath(fifthPath); defineFirstServoPoses(hardwareMap);break;
            case CLIP3: defineClipSetServoPoses(hardwareMap); break;
            case COMPLETE: stop(); break;

        }
    }

    private PathChain buildFirstPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.000, 58.000, Point.CARTESIAN),
                                new Point(39.2, 76.00, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private PathChain buildSecondPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(39.200, 76.000, Point.CARTESIAN),
                                new Point(9.000, 24.000, Point.CARTESIAN)
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
                                new Point(9.000, 24.000, Point.CARTESIAN),
                                new Point(39.200, 74.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private PathChain buildFourthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(39.200, 74.000, Point.CARTESIAN),
                                new Point(9.000, 24.000, Point.CARTESIAN)

                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    private PathChain buildFifthPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(9.000, 24.000, Point.CARTESIAN),
                                new Point(39.200, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }


    private PathChain ParkPath(){
        return follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(28.747, 71.226, Point.CARTESIAN),
                                new Point(0, 53, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
    }

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
