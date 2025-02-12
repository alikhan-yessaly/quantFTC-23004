package org.firstinspires.ftc.teamcode.notcompetition.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.utils.ArmTPD;
import org.firstinspires.ftc.teamcode.utils.Extender;
import org.firstinspires.ftc.teamcode.utils.ExtenderPD;
import org.firstinspires.ftc.teamcode.utils.LiftPD;
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
import java.util.Queue;
import java.util.LinkedList;

@Autonomous(name = "Dava3Basket", group = "Autonomous")
public class HighBasket3 extends OpMode{

    private enum AutoState{
        INITIALIZE, FIRST_PATH, SECOND_PATH, THIRD_PATH, FOURTH_PATH, FIFTH_PATH, SIXTH_PATH, SEVENTH_PATH, PICK_POSE, TRANSFER_POSE, BASKET_POSE, RETURN_POSE, ARM_UP, ARM_DOWN, EXTEND_OUT, EXTEND_IN, LIFT_UP, LIFT_DOWN, LIFT_TRANSFER, LIFT_TAKE, PARK_PATH, COMPLETE;
    }

    private AutoState currentState = AutoState.INITIALIZE;
    private final Timer opmodeTimer = new Timer();
    private Follower follower;
    private ServoPoseFollower servoPoseFollower;
    private ArmTPD armT;
    private ExtenderPD extender;
    private LiftPD lifts;

    private Queue<AutoState> stateQueue = new LinkedList<>();

    private PathChain firstPath, secondPath, thirdPath, fourthPath, fifthPath, sixthPath, seventhPath;
    public static double armT_kD = 0.00001, armT_kP = 0.001;
    public static double lift_kD = 0.0001, lift_kP = 0.001;
    public static double extender_Kd = 0.00001, extender_Kp = 0.0005;

    private static final Pose START_POSE = new Pose(9.00, 85.00, 0);

    @Override
    public void init(){
        follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE);

        armT = new ArmTPD(hardwareMap);
        extender = new ExtenderPD(hardwareMap);
        lifts = new LiftPD(hardwareMap);

        defineInitialServoPoses(hardwareMap);
        servoPoseFollower.start();

        armT.resetEncoder();
        extender.resetEncoder();
        lifts.resetEncoder();

        telemetry.addData("Init Status", "Initialized with starting pose and servo positions.");
        telemetry.update();
    }

    @Override
    public void start(){
        firstPath = buildFirstPath();
        secondPath = secondPath();
        thirdPath = thirdPath();
        fourthPath = fourthPath();
        fifthPath = fifthPath();
        sixthPath = sixthPath();
        seventhPath = sevensPath();

       stateQueue.add(AutoState.LIFT_TRANSFER); //1st basket
       stateQueue.add(AutoState.FIRST_PATH);
       stateQueue.add(AutoState.EXTEND_OUT);
       stateQueue.add(AutoState.PICK_POSE);
       stateQueue.add(AutoState.EXTEND_IN);
       stateQueue.add(AutoState.SECOND_PATH);
       stateQueue.add(AutoState.LIFT_TAKE);
       stateQueue.add(AutoState.TRANSFER_POSE);
       stateQueue.add(AutoState.LIFT_UP);
       stateQueue.add(AutoState.ARM_UP);
       stateQueue.add(AutoState.BASKET_POSE);
       stateQueue.add(AutoState.ARM_DOWN);
       stateQueue.add(AutoState.LIFT_TRANSFER);  //2nd basket
       stateQueue.add(AutoState.THIRD_PATH);
       stateQueue.add(AutoState.EXTEND_OUT);
       stateQueue.add(AutoState.PICK_POSE);
       stateQueue.add(AutoState.EXTEND_IN);
       stateQueue.add(AutoState.FOURTH_PATH);
       stateQueue.add(AutoState.LIFT_TAKE);
       stateQueue.add(AutoState.TRANSFER_POSE);
       stateQueue.add(AutoState.LIFT_UP);
       stateQueue.add(AutoState.ARM_UP);
       stateQueue.add(AutoState.BASKET_POSE);
       stateQueue.add(AutoState.ARM_DOWN);
       stateQueue.add(AutoState.LIFT_TRANSFER); // 3rd Basket
       stateQueue.add(AutoState.FIFTH_PATH);
       stateQueue.add(AutoState.EXTEND_OUT);
       stateQueue.add(AutoState.PICK_POSE);
       stateQueue.add(AutoState.EXTEND_IN);
       stateQueue.add(AutoState.SIXTH_PATH);
       stateQueue.add(AutoState.LIFT_TAKE);
       stateQueue.add(AutoState.TRANSFER_POSE);
       stateQueue.add(AutoState.LIFT_UP);
       stateQueue.add(AutoState.ARM_UP);
       stateQueue.add(AutoState.BASKET_POSE);
       stateQueue.add(AutoState.ARM_DOWN);
       stateQueue.add(AutoState.LIFT_DOWN);
       stateQueue.add(AutoState.SEVENTH_PATH); //return to starting point
       stateQueue.add(AutoState.RETURN_POSE);
       stateQueue.add(AutoState.COMPLETE);

       setNextState();
    }

    @Override
    public void loop(){
        armT.setPID(armT_kP, 0, armT_kD);
        extender.setPID(extender_Kp, 0, extender_Kd);
        lifts.setPID(lift_kP,0,lift_kD);
        follower.update();
        servoPoseFollower.update();
        if (currentState == null && !stateQueue.isEmpty()) {
            setNextState();
        }

        if(currentState != null){
            switch (currentState){
                // Grouped follower-based states
                case FIRST_PATH: case SECOND_PATH: case THIRD_PATH: case FOURTH_PATH: case FIFTH_PATH: case SIXTH_PATH: case SEVENTH_PATH:
                    if (follower.isCloseEnoughToEnd()) {
                        setNextState();
                    }
                    break;

                // Grouped Motor-Based States
                case ARM_UP: case ARM_DOWN: case LIFT_DOWN: case LIFT_TAKE: case LIFT_UP: case LIFT_TRANSFER:  case EXTEND_OUT: case EXTEND_IN:
                    setNextState();
                    break;
                    
                    // Grouped servo-based states
                case PICK_POSE: case TRANSFER_POSE: case BASKET_POSE: case RETURN_POSE:
                    if (servoPoseFollower.isComplete()) {
                        setNextState();
                    }
                    break;

                case COMPLETE:
                    telemetry.addData("Status", "Autonomous Complete");
                    break;

            }

        }

        telemetry.addData("Current State", currentState);
        telemetry.addData("Follower X", follower.getPose().getX());
        telemetry.addData("Follower Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Timer", opmodeTimer.getElapsedTime());
        telemetry.update();
    }

    @Override
    public void stop(){
        telemetry.addData("Status", "Autonomous Stopped");
        telemetry.update();
    }

    private void setNextState() {
        if (!stateQueue.isEmpty()) {
            currentState = stateQueue.poll();
            opmodeTimer.resetTimer();
            applyStateAction(currentState);
        } else {
            currentState = null;
        }
    }

    private void applyStateAction(AutoState newState){
        currentState = newState;
        opmodeTimer.resetTimer();
        switch(newState){
            case FIRST_PATH: follower.followPath(firstPath); break;
            case SECOND_PATH: follower.followPath(secondPath); break;
            case PICK_POSE: definePickServoPoses(hardwareMap); servoPoseFollower.start(); break;
            case TRANSFER_POSE: defineTransferServoPoses(hardwareMap); servoPoseFollower.start(); break;
            case BASKET_POSE: defineBasketServoPoses(hardwareMap); servoPoseFollower.start(); break;
            case RETURN_POSE: defineReturnServoPoses(hardwareMap); servoPoseFollower.start(); break;
            case ARM_UP: armT.setTargetPosition(3000); break;
            case ARM_DOWN: armT.setTargetPosition(0); break;
            case EXTEND_OUT: extender.setTargetPosition(-1500); break;
            case EXTEND_IN: extender.setTargetPosition(0); break;
            case LIFT_UP: lifts.moveUp(); break;
            case LIFT_DOWN: lifts.moveDown(); break;
            case LIFT_TRANSFER: lifts.setTargetPosition(-700); break;
            case LIFT_TAKE: lifts.setTargetPosition(0); break;
        }
    }
    //===================================================== PATHS PART ==================================================================
    private PathChain buildFirstPath(){
        return follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.000, 85.000, Point.CARTESIAN),
                                new Point(18.635, 119.718, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    private PathChain secondPath(){
        return follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(18.635, 119.718, Point.CARTESIAN),
                                new Point(17.506, 124.800, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();
    }
    private PathChain thirdPath(){
        return follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(17.506, 124.800, Point.CARTESIAN),
                                new Point(18.635, 130.447, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();
    }
    private PathChain fourthPath(){
        return follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(18.635, 130.447, Point.CARTESIAN),
                                new Point(17.506, 124.800, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();
    }
    private PathChain fifthPath(){
        return follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(17.506, 124.800, Point.CARTESIAN),
                                new Point(18.071, 130.447, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(30))
                .build();
    }
    private PathChain sixthPath(){
        return follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(18.071, 130.447, Point.CARTESIAN),
                                new Point(17.506, 124.800, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(-45))
                .build();
    }
    private PathChain sevensPath(){
        return follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(17.506, 124.800, Point.CARTESIAN),
                                new Point(9.000, 85.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();
    }
    //======================================================= SERVO SETTING PART =======================================================================
    private void defineInitialServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> initialPoses = Arrays.asList(
                new ServoPose(0.35, 0.35, 0.85, 0, 1,100)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
    }
    private void definePickServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> pickPoses = Arrays.asList(
                new ServoPose(0.35, 0.35, 0.85, 1, 0.4,300),
                new ServoPose(0.35, 0.35, 0.85, 1, 0,300),
                new ServoPose(0.35, 0.65, 0.85, 1, 0.4,300),
                new ServoPose(0.35, 0.65, 0.85, 1, 0.4,200),
                new ServoPose(0.35, 0.65, 0.85, 0.6, 0.6,200),
                new ServoPose(0.35, 0.65, 0.85, 0.2, 0.8,200),
                new ServoPose(0.35, 0.65, 0.85, 0, 1,200)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, pickPoses);
    }

    private void defineTransferServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> transferPoses = Arrays.asList(
                new ServoPose(0.65, 0.35, 0.85, 0, 1,100)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, transferPoses);
    }

    private void defineBasketServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> basketPoses = Arrays.asList(
                new ServoPose(0.65, 0.35, 0.85, 0, 1,200),
                new ServoPose(0.65, 0.35, 0.85, 0.2, 1,200),
                new ServoPose(0.65, 0.35, 0.85, 0.4, 1,200),
                new ServoPose(0.65, 0.35, 0.85, 0.6, 1,200),
                new ServoPose(0.65, 0.35, 0.85, 0.8, 1,200),
                new ServoPose(0.65, 0.35, 0.85, 1, 1,200),
                new ServoPose(0.35, 0.35, 0.85, 1, 1,200)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, basketPoses);
    }

    private void defineReturnServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> returnPoses = Arrays.asList(
                new ServoPose(0.35, 0.35, 0.85, 1, 1,200),
                new ServoPose(0.35, 0.35, 0.85, 0.8, 1,200),
                new ServoPose(0.35, 0.35, 0.85, 0.6, 1,200),
                new ServoPose(0.35, 0.35, 0.85, 0.4, 1,200),
                new ServoPose(0.35, 0.35, 0.85, 0.2, 1,200),
                new ServoPose(0.35, 0.35, 0.85, 0, 1,200)

        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, returnPoses);
    }

}
