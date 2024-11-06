package org.firstinspires.ftc.teamcode.notcompetition.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

@Autonomous(name = "TestPedroAuto", group = "Autonomous")
public class TestPedroAuto extends OpMode {

    // Private enum to define different autonomous states
    @SuppressWarnings("unused")
    private enum AutoState {
        START,
        FIRST_POSE,
        DUMMY_PATH,
        SECOND_POSE,
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
                new ServoPose(0.5, 0.7, 0.7, 1.0, 0.0, 1000),
                new ServoPose(1.0, 0.5, 0.5, 0.5, 1.0, 1500)
        );
        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
    }

    public void defineSecondPoseSequence() {
        List<ServoPose> secondPoses = Arrays.asList(
                new ServoPose(0.3, 0.6, 0.6, 0.9, 0.0, 1200),
                new ServoPose(1.0, 0.8, 0.8, 1.0, 1.0, 1500)
        );
        servoPoseFollower.setPoseSequence(secondPoses); // Load the new sequence
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        scanTimer = new Timer();

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
                follower.followPath(firstCycle); // Start path following
                servoPoseFollower.start();       // Start servo pose follower
                break;
            case FIRST_POSE:
                defineInitialServoPoses(hardwareMap);
                servoPoseFollower.start();       // Start new servo sequence
                break;
            case DUMMY_PATH:
                follower.followPath(dummyPath);  // Start dummy path after first pose
                break;
            case SECOND_POSE:
                defineSecondPoseSequence();
                servoPoseFollower.start();
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
                if (follower.isCloseEnoughToEnd()) {
                    setPathState(AutoState.FIRST_POSE); // Move to FIRST_POSE checkpoint
                }
                break;

            case FIRST_POSE:
                if (servoPoseFollower.isComplete()) {
                    setPathState(AutoState.DUMMY_PATH); // Transition to DUMMY_PATH after FIRST_POSE
                }
                break;

            case DUMMY_PATH:
                if (follower.isCloseEnoughToEnd()) {
                    setPathState(AutoState.SECOND_POSE); // Move to SECOND_POSE after completing dummy path
                }
                break;

            case SECOND_POSE:
                if (servoPoseFollower.isComplete()) {
                    setPathState(AutoState.FINAL_POSE); // Move to FINAL_POSE checkpoint
                }
                break;

            case FINAL_POSE:
                if (servoPoseFollower.isComplete()) {
                    setPathState(AutoState.COMPLETE); // Finalize actions
                }
                break;

            case COMPLETE:
                telemetry.addData("Status", "Sequence Complete");
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
