package org.firstinspires.ftc.teamcode.notcompetition.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.utils.ServoPose;
import org.firstinspires.ftc.teamcode.utils.ServoPoseFollower;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "TestPedroAuto", group = "Autonomous")
public class TestPedroAuto extends OpMode {

    private DcMotor armLift1, armLift2;

    // Private enum to define different autonomous states
    @SuppressWarnings("unused")
    private enum AutoState {
        START,
        FIRST_POSE,
        LIFT_UP,
        SECOND_POSE,
        LIFT_DOWN,
        FINAL_POSE,
        COMPLETE
    }

    private Timer pathTimer, opmodeTimer, scanTimer;
    private Follower follower;
    private PathChain firstCycle;
    private PathChain dummyPath; // New dummy path
    private ServoPoseFollower servoPoseFollower;
    private AutoState currentState = AutoState.START; // Initial state

    // Constants for starting pose
    private static final Pose START_POSE = new Pose(144 - (63 + 72), 12 + 72, 0);




    public void buildPaths() {
        // Initial path (firstCycle)
        firstCycle = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(7.182, 90.359, Point.CARTESIAN),
                                new Point(33.890, 92.603, Point.CARTESIAN),
                                new Point(16.833, 74.873, Point.CARTESIAN),
                                new Point(39.277, 75.546, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        // Dummy path for testing purposes
        dummyPath = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(39.277, 75.546, Point.CARTESIAN),
                                new Point(60.0, 80.0, Point.CARTESIAN), // Dummy target point
                                new Point(80.0, 60.0, Point.CARTESIAN),
                                new Point(100.0, 75.0, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public void defineInitialServoPoses(HardwareMap hardwareMap) {
        List<ServoPose> initialPoses = Arrays.asList(
                new ServoPose(0, 0, 0, 0.5, 0.45, 1000)//,
//                new ServoPose(1.0, 0.5, 0.5, 0.5, 1.0, 1500)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
    }

    public void defineSecondPoseSequence() {
        List<ServoPose> secondPoses = Arrays.asList(
                new ServoPose(1, 1, 0.5, 0.5, 0.45, 1200)
                //new ServoPose(1.0, 0.8, 0.8, 1.0, 1.0, 1500)
        );
        servoPoseFollower.setPoseSequence(secondPoses); // Load the new sequence
    }
    public void defineThirdPoseSequence() {
        List<ServoPose> ThirdPoses = Arrays.asList(
                new ServoPose(1, 1, 0.5, 0.5, 0.0, 1200)
        );
        servoPoseFollower.setPoseSequence(ThirdPoses); // Load the new sequence
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        scanTimer = new Timer();

        armLift1 = hardwareMap.dcMotor.get("lift1");
        armLift2 = hardwareMap.dcMotor.get("lift2");
        armLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLift2.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE);

        buildPaths();
        defineInitialServoPoses(hardwareMap); // Define the first set of poses

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        scanTimer.resetTimer();
    }

    @Override
    public void start() {
        servoPoseFollower.start();
        opmodeTimer.resetTimer();
        setPathState(AutoState.START);
    }

    private void setPathState(AutoState newState) {
        currentState = newState;
        pathTimer.resetTimer();

        switch (currentState) {
            case START:
                defineInitialServoPoses(hardwareMap);
                servoPoseFollower.start();
                break;
            case FIRST_POSE:
                follower.followPath(firstCycle);
                break;
            case LIFT_UP:
                armLift1.setTargetPosition(-1000);
                armLift2.setTargetPosition(-1000);
                armLift1.setPower(1);
                armLift2.setPower(1);
                armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case SECOND_POSE:
                defineSecondPoseSequence();
                servoPoseFollower.start();
                break;
            case LIFT_DOWN:
                armLift1.setTargetPosition(-1000);
                armLift2.setTargetPosition(-1000);
                armLift1.setPower(1);
                armLift2.setPower(1);
                armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case FINAL_POSE:
                // Define any final poses or actions here
                break;
            case COMPLETE:
                // Stop all movement or finalize actions
                stop();
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        servoPoseFollower.update();

        // State transitions based on follower position and servo pose completion
        switch (currentState) {
            case START:
                setPathState(AutoState.START);// Transition to DUMMY_PATH after FIRST_POSE
                break;

            case FIRST_POSE:
                if (follower.isCloseEnoughToEnd()) {
                    setPathState(AutoState.FIRST_POSE); // Move to SECOND_POSE after completing dummy path
                }
                break;

            case LIFT_UP:
                setPathState(AutoState.LIFT_UP);
                break;

            case LIFT_DOWN:
                setPathState(AutoState.LIFT_DOWN); // Move to FINAL_POSE checkpoint
                break;

            case FINAL_POSE:
                break;

            case COMPLETE:
                telemetry.addData("Status", "Sequence Complete");
                telemetry.update();
                break;
        }

        // Optional telemetry for debugging
        telemetry.addData("Current State", currentState);
        telemetry.addData("Follower X", follower.getPose().getX());
        telemetry.addData("Follower Y", follower.getPose().getY());
        telemetry.addData("Servo Pose Complete", servoPoseFollower.isComplete());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
