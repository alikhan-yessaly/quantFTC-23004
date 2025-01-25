package org.firstinspires.ftc.teamcode.notcompetition.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.utils.Arm;
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


@Autonomous(name = "BlueClip", group = "Autonomous")
public class BlueClip extends OpMode{

    private enum AutoState {
        INITIALIZE, FIRST_PATH, SECOND_PATH, THIRD_PATH, FIRST_POSE, SECOND_POSE, THIRD_POSE, LIFT_UP, LIFT_DOWN, RETURN_POSE;
    }

    private AutoState currentState = AutoState.INITIALIZE;
    private Timer opmodeTimer = new Timer();
    private Follower follower;
    private ServoPoseFollower servoPoseFollower;
    private ArmLift armLift;

    private PathChain firstPath, secondPath, thirdPath, firstPose, secondPose, liftUp, thirdPose, liftDown, returnPose;

    private static final Pose START_POSE = new Pose(9.00, 58.00, Math.toRadians(180));

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE);

        armLift = new ArmLift(hardwareMap);
        armLift.moveDown();

        defineInitialServoPoses(hardwareMap);
        servoPoseFollower.start();

        telemetry.addData("Init Status", "Initialized with starting pose and servo positions.");
        telemetry.update();

    }

    @Override
    public void start(){
        firstPath = buildFirstPath();
        secondPath = SecondPath();
        thirdPath = ThirdPath();

        setState(AutoState.FIRST_PATH);

    }

    @Override
    public void loop(){
        follower.update();
        servoPoseFollower.update();

        switch (currentState){
            case FIRST_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.FIRST_POSE);
                break;
            case FIRST_POSE:
                if(servoPoseFollower.isComplete()) setState(AutoState.SECOND_PATH);

         }

        telemetry.addData("Current State", currentState);
        telemetry.addData("Follower X", follower.getPose().getX());
        telemetry.addData("Follower Y", follower.getPose().getY());
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
            case FIRST_PATH: follower.followPath(firstPath); break;
            case SECOND_PATH: follower.followPath(secondPath); break;
            case THIRD_PATH: follower.followPath(thirdPath); break;
            case LIFT_DOWN: armLift.moveDown(); break;
            case LIFT_UP: armLift.moveUp(); break;
            case FIRST_POSE: defineInitialServoPoses(hardwareMap); servoPoseFollower.start(); break;

        }
    }

    private PathChain buildFirstPath() {
        return follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.000, 58.000, Point.CARTESIAN),
                                new Point(36.000, 58.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private void defineInitialServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> initialPoses = Arrays.asList(
             new ServoPose(0.65, 0.35, 0.55, 0.3, 0.15, 0.88, 0.25,1000)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
    }

    private PathChain SecondPath(){
        return follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(39.476, 58.095, Point.CARTESIAN),
                                new Point(30.280, 31.180, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-35))
                .build();
    }

    private PathChain ThirdPath(){
        return follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(30.280, 31.180, Point.CARTESIAN),
                                new Point(30.280, 31.180, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(-150))
                .build();

    }



}
