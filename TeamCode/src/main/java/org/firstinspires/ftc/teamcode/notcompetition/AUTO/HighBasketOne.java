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

@Autonomous(name = "Dava1Basket", group = "Autonomous")
public class HighBasketOne extends OpMode{

    private enum AutoState{
        INITIALIZE, FIRST_PATH, SECOND_PATH, PICK_POSE, TRANSFER_POSE, BASKET_POSE, RETURN_POSE, ARM_UP, ARM_DOWN, EXTEND_OUT, EXTEND_IN, LIFT_UP, LIFT_DOWN, LIFT_TRANSFER, LIFT_TAKE, PARK_PATH, COMPLETE;
    }

    private AutoState currentState = AutoState.INITIALIZE;
    private final Timer opmodeTimer = new Timer();
    private Follower follower;
    private ServoPoseFollower servoPoseFollower;
    private ArmTPD armT;
    private ExtenderPD extender;
    private LiftPD lifts;

    private PathChain firstPath, secondPath;
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

        telemetry.addData("Init Status", "Initialized with starting pose and servo positions.");
        telemetry.update();
    }

    @Override
    public void start(){
        firstPath = buildFirstPath();
        secondPath = secondPath();

        setState(AutoState.LIFT_TRANSFER);
    }

    @Override
    public void loop(){
        armT.setPID(armT_kP, 0, armT_kD);
        extender.setPID(extender_Kp, 0, extender_Kd);
        lifts.setPID(lift_kP,0,lift_kD);
        follower.update();
        servoPoseFollower.update();
        switch (currentState){
            case LIFT_TRANSFER:
                setState(AutoState.FIRST_PATH);
                break;
            case FIRST_PATH:
                if(servoPoseFollower.isComplete()) setState(AutoState.EXTEND_OUT);
                break;
            case EXTEND_OUT:
                setState(AutoState.PICK_POSE);
                break;
            case PICK_POSE:
                if(servoPoseFollower.isComplete()) setState(AutoState.EXTEND_IN);
                break;
            case EXTEND_IN:
                setState(AutoState.SECOND_PATH);
                break;
            case SECOND_PATH:
                if(follower.isCloseEnoughToEnd()) setState(AutoState.LIFT_TAKE);
                break;
            case LIFT_TAKE:
                setState(AutoState.TRANSFER_POSE);
                break;
            case TRANSFER_POSE:
                if(servoPoseFollower.isComplete()) setState(AutoState.LIFT_UP);
                break;
            case LIFT_UP:
                 setState(AutoState.ARM_UP);
                break;
            case ARM_UP:
                setState(AutoState.BASKET_POSE);
            case BASKET_POSE:
                if(servoPoseFollower.isComplete()) setState(AutoState.ARM_DOWN);
                break;
            case ARM_DOWN:
                setState(AutoState.LIFT_DOWN);
                break;
            case LIFT_DOWN:
                setState(AutoState.RETURN_POSE);
                break;
            case RETURN_POSE:
                if(servoPoseFollower.isComplete()) setState(AutoState.COMPLETE);
            case COMPLETE:
                telemetry.addData("Status", "Autonomous Complete");
                break;
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

    private void setState(AutoState newState){
        currentState = newState;
        opmodeTimer.resetTimer();
        switch(newState){
            case FIRST_PATH: follower.followPath(firstPath); break;
            case SECOND_PATH: follower.followPath(secondPath); break;
            case PICK_POSE: definePickServoPoses(hardwareMap); servoPoseFollower.start(); break;
            case TRANSFER_POSE: defineTransferServoPoses(hardwareMap); servoPoseFollower.start(); break;
            case BASKET_POSE: defineBasketServoPoses(hardwareMap); servoPoseFollower.start(); break;
            case RETURN_POSE: defineReturnServoPoses(hardwareMap); servoPoseFollower.start(); break;
            case ARM_UP: armT.setTargetPosition(4000); break;
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
                                new Point(18.635, 120.282, Point.CARTESIAN)
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
                                new Point(18.635, 120.282, Point.CARTESIAN),
                                new Point(18.071, 125.929, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
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
                new ServoPose(0.35, 0.65, 0.85, 1, 0.4,300)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, pickPoses);
    }

    private void defineTransferServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> transferPoses = Arrays.asList(
                new ServoPose(0.35, 0.65, 0.85, 1, 0.4,200),
                new ServoPose(0.35, 0.65, 0.85, 0.6, 0.6,200),
                new ServoPose(0.35, 0.65, 0.85, 0.2, 0.8,200),
                new ServoPose(0.35, 0.65, 0.85, 0, 1,200),
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
