//package org.firstinspires.ftc.teamcode.notcompetition.AUTO;
//
//import static org.firstinspires.ftc.teamcode.utils.ArmLift.CLIP_DOWN_POSITION;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.utils.ArmTPD;
//import org.firstinspires.ftc.teamcode.utils.ArmLift;
//import org.firstinspires.ftc.teamcode.utils.ServoPose;
//import org.firstinspires.ftc.teamcode.utils.ServoPoseFollower;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//
//
//@Autonomous(name = "TwoClipTry", group = "Autonomous")
//public class TwoClipTry extends OpMode{
//
//    private enum AutoState {
//        INITIALIZE, FIRST_PATH, CLIP1, CLIP2, SECOND_PATH, THIRD_PATH, FOURTH_PATH, FIFTH_PATH, FIRST_POSE, SECOND_POSE, THIRD_POSE, FOURTH_POSE,  LIFT_UP, LIFT_DOWN, PARK_PATH, PARK_POSE,COMPLETE;
//    }
//
//    private AutoState currentState = AutoState.INITIALIZE;
//    private Timer opmodeTimer = new Timer();
//    private Follower follower;
//    private ServoPoseFollower servoPoseFollower;
//    private ArmLift armLift;
//
//    private PathChain firstPath, secondPath, thirdPath, fourthPath, fifthPath, sixthPath, firstPose, secondPose, liftUp, thirdPose, liftDown, parkPath;
//
//    private static final Pose START_POSE = new Pose(9.00, 58.00, Math.toRadians(180));
//
//    @Override
//    public void init() {
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(START_POSE);
//
//        armLift = new ArmLift(hardwareMap);
//        armLift.moveDown();
//
//        defineInitialServoPoses(hardwareMap);
//        servoPoseFollower.start();
//
//        telemetry.addData("Init Status", "Initialized with starting pose and servo positions.");
//        telemetry.update();
//
//    }
//
//    @Override
//    public void start(){
//        firstPath = buildFirstPath();
//        secondPath = SecondPath();
//        thirdPath = ThirdPath();
//        fourthPath = FourthPath();
//        fifthPath = FifthPath();
//        parkPath = ParkPath();
//
//        setState(AutoState.FIRST_POSE);
//
//    }
//
//    @Override
//    public void loop(){
//        follower.update();
//        servoPoseFollower.update();
//        switch (currentState){
//            case FIRST_POSE:
//                if(servoPoseFollower.isComplete()) setState(AutoState.LIFT_UP);
//                break;
//            case LIFT_UP:
//                setState(AutoState.FIRST_PATH);
//                break;
//            case FIRST_PATH:
//                if(follower.isCloseEnoughToEnd()) setState(AutoState.LIFT_DOWN);
//                break;
//            case LIFT_DOWN:
//                setState(AutoState.SECOND_POSE);
//                break;
//            case SECOND_POSE:
//                if(servoPoseFollower.isComplete()) setState(AutoState.SECOND_PATH);
//                break;
//            case SECOND_PATH:
//                if(follower.isCloseEnoughToEnd()) setState(AutoState.THIRD_PATH);
//                break;
//            case THIRD_PATH:
//                if(follower.isCloseEnoughToEnd()) setState(AutoState.FOURTH_PATH);
//                break;
//            case FOURTH_PATH:
//                if(follower.isCloseEnoughToEnd()) setState(AutoState.THIRD_POSE);
//            case THIRD_POSE:
//                if(servoPoseFollower.isComplete()) setState(AutoState.CLIP1);
//                break;
//            case CLIP1:
//                setState(AutoState.FOURTH_POSE);
//                break;
//            case FOURTH_POSE:
//                if(servoPoseFollower.isComplete()) setState(AutoState.CLIP2);
//                break;
//            case CLIP2:
//                setState(AutoState.PARK_PATH);
//                break;
//            case PARK_PATH:
//                if(follower.isCloseEnoughToEnd()) setState(AutoState.PARK_POSE);
//                break;
//            case PARK_POSE:
//                if(servoPoseFollower.isComplete()) setState(AutoState.COMPLETE);
//            case COMPLETE:
//                telemetry.addData("Status", "Autonomous Complete");
//                break;
//        }
//
//        telemetry.addData("Current State", currentState);
//        telemetry.addData("Follower X", follower.getPose().getX());
//        telemetry.addData("Follower Y", follower.getPose().getY());
//        telemetry.addData("Heading", Math.toRadians(follower.getPose().getHeading()));
//        telemetry.addData("Lifts", armLift.getCurrentPosition());
//        telemetry.update();
//    }
//
//    @Override
//    public void stop(){
//        telemetry.addData("Status", "Autonomous Stopped");
//        telemetry.update();
//    }
//
//    private void setState(AutoState newState){
//        currentState = newState;
//        opmodeTimer.resetTimer();
//        switch (newState) {
//            case FIRST_PATH: follower.followPath(firstPath); break;
//            case CLIP2: armLift.moveClipDown(); break;
//            case CLIP1: armLift.moveClipUp(); break;
//            case PARK_PATH: follower.followPath(parkPath); break;
//            case THIRD_PATH: follower.followPath(thirdPath); break;
//            case FOURTH_PATH: follower.followPath(fourthPath); break;
//            case FIFTH_PATH: follower.followPath(fifthPath); break;
//            case LIFT_DOWN: armLift.moveDown(); break;
//            case LIFT_UP: armLift.moveUp(); break;
//            case PARK_POSE: defineParkServoPoses(hardwareMap); servoPoseFollower.start(); break;
//            case FIRST_POSE: defineInitialServoPoses(hardwareMap); servoPoseFollower.start(); break;
//            case SECOND_POSE: defineSecondServoPoses(hardwareMap); servoPoseFollower.start(); break;
//            case THIRD_POSE: defineThirdServoPoses(hardwareMap); servoPoseFollower.start(); break;
//            case FOURTH_POSE: defineFourthServoPoses(hardwareMap); servoPoseFollower.start(); break;
//        }
//    }
//
//    private PathChain buildFirstPath() {
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 1
//                        new BezierLine(
//                                new Point(9.000, 58.000, Point.CARTESIAN),
//                                new Point(28.747, 71.226, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//    }
//
//
//    private PathChain ParkPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line Park
//                        new BezierLine(
//                                new Point(28.747, 71.226, Point.CARTESIAN),
//                                new Point(0, 58, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
//                .build();
//    }
//
//    private void defineInitialServoPoses(HardwareMap hardwareMap) {
//        List<ServoPose> initialPoses = Arrays.asList(
//                new ServoPose(0.65, 0.35, 0.55, 0.3, 1, 0.18, 0.25,100),
//                new ServoPose(0.65, 0.35, 0.55, 0.3, 1, 0.28, 0.25,100),
//                new ServoPose(0.65, 0.35, 0.55, 0.3, 1, 0.38, 0.25,100),
//                new ServoPose(0.65, 0.35, 0.55, 0.3, 1, 0.48, 0.25,100),
//                new ServoPose(0.65, 0.35, 0.55, 0.3, 1, 0.58, 0.25,100),
//                new ServoPose(0.65, 0.35, 0.55, 0.3, 1, 0.68, 0.25,100),
//                new ServoPose(0.65, 0.35, 0.55, 0.3, 1, 0.78, 0.25,100),
//                new ServoPose(0.65, 0.35, 0.55, 0.3, 1, 0.88, 0.25,100)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
//    }
//
//
//    private void defineSecondServoPoses(HardwareMap hardwareMap){
//        List<ServoPose> secondPoses = Arrays.asList(
//                new ServoPose(0.35, 0.35, 0.55, 0.3, 1, 0.88, 0.25, 100)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, secondPoses);
//    }
//
//    private void defineParkServoPoses(HardwareMap hardwareMap){
//        List<ServoPose> parkPoses = Arrays.asList(
//                new ServoPose(0.35, 0.35, 0.5,0.3,1, 0.25, 0.25, 100)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, parkPoses);
//    }
//
//    private void defineThirdServoPoses(HardwareMap hardwareMap){
//        List<ServoPose> thirdPoses = Arrays.asList(
//                new ServoPose(0.65, 0.35, 0.5,0.3,1, 0.88, 0.25, 100)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, thirdPoses);
//    }
//
//    private void defineFourthServoPoses(HardwareMap hardwareMap){
//        List<ServoPose> fourthPoses = Arrays.asList(
//                new ServoPose(0.35, 0.35, 0.5,0.3,1, 0.88, 0.25, 100)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, fourthPoses);
//    }
//
//    private PathChain SecondPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 2
//                        new BezierLine(
//                                new Point(28.747, 71.226, Point.CARTESIAN),
//                                new Point(16.149, 128.748, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
//                .build();
//
//    }
//
//    private PathChain ThirdPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 3
//                        new BezierLine(
//                                new Point(16.149, 128.748, Point.CARTESIAN),
//                                new Point(9.000, 128.524, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//    }
//
//    private PathChain FourthPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 4
//                        new BezierLine(
//                                new Point(9.000, 128.524, Point.CARTESIAN),
//                                new Point(16.149, 128.748, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//    }
//
//    private PathChain FifthPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 5
//                        new BezierLine(
//                                new Point(16.149, 128.748, Point.CARTESIAN),
//                                new Point(39.700, 63.927, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
//                .build();
//
//    }
//
//
//
//}
