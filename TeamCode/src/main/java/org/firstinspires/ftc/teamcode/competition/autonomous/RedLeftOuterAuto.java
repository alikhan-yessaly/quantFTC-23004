package org.firstinspires.ftc.teamcode.competition.autonomous;


import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_AUTO_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSE_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.LIFT_TRANSFER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_ARM_YELLOW_SCORE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_CLAW_DROP_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ROBOT_FRONT_LENGTH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_RESET;
import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.competition.teleop.TwoPersonDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.VisionPortalTeamPropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Left Outer Auto", group = "Autonomous")
public class RedLeftOuterAuto extends OpMode {

    private TwoPersonDrive twoPersonDrive;

    private Timer pathTimer, opmodeTimer, scanTimer;

    private VisionPortalTeamPropPipeline teamPropPipeline;

    private VisionPortal visionPortal;

    private String navigation;

    private DistanceSensor leftDistanceSensor, rightDistanceSensor;

    private RevColorSensorV3 colorSensor;

    private boolean distanceSensorDisconnected, colorSensorDisconnected;

    // IMPORTANT: y increasing is towards the backstage from the audience,
    // while x increasing is towards the red side from the blue side
    // this means that 0 heading is pointing from the blue side to the red side

    // all spike mark locations since I'm lazy
    private Pose2d redLeftSideLeftSpikeMark = new Pose2d(36 + 72, -47.5 + 72);
    private Pose2d redLeftSideMiddleSpikeMark = new Pose2d(24.5 + 72, -36 + 72);
    private Pose2d redLeftSideRightSpikeMark = new Pose2d(36 + 72, -24.5 + 72);
    private Pose2d redRightSideLeftSpikeMark = new Pose2d(36 + 72, 0.5 + 72);
    private Pose2d redRightSideMiddleSpikeMark = new Pose2d(24.5 + 72, 12 + 72);
    private Pose2d redRightSideRightSpikeMark = new Pose2d(36 + 72, 23.5 + 72);
    private Pose2d blueLeftSideLeftSpikeMark = new Pose2d(-36 + 72, 23.5 + 72);
    private Pose2d blueLeftSideMiddleSpikeMark = new Pose2d(-24.5 + 72, 12 + 72);
    private Pose2d blueLeftSideRightSpikeMark = new Pose2d(-36 + 72, 0.5 + 72);
    private Pose2d blueRightSideLeftSpikeMark = new Pose2d(-36 + 72, -24.5 + 72);
    private Pose2d blueRightSideMiddleSpikeMark = new Pose2d(-24.5 + 72, -36 + 72);
    private Pose2d blueRightSideRightSpikeMark = new Pose2d(-36 + 72, -47.5 + 72);

    // backdrop april tag locations
    private Pose2d blueLeftBackdrop = new Pose2d(-42.875 + 72, 60.75 + 72);
    private Pose2d blueMiddleBackdrop = new Pose2d(-36.75 + 72, 60.75 + 72);
    private Pose2d blueRightBackdrop = new Pose2d(-30.75 + 72, 60.75 + 72);
    private Pose2d redLeftBackdrop = new Pose2d(30.75 + 72, 60.75 + 72);
    private Pose2d redMiddleBackdrop = new Pose2d(36.75 + 72, 60.75 + 72);
    private Pose2d redRightBackdrop = new Pose2d(42.875 + 72, 60.75 + 72);

    // white pixel stack locations
    private Pose2d redOuterStack = new Pose2d(36 + 72, -72 + 72);
    private Pose2d redMiddleStack = new Pose2d(24 + 72, -72 + 72);
    private Pose2d redInnerStack = new Pose2d(12 + 72, -72 + 72);
    private Pose2d blueInnerStack = new Pose2d(-12 + 72, -72 + 72);
    private Pose2d blueMiddleStack = new Pose2d(-24 + 72, -72 + 72);
    private Pose2d blueOuterStack = new Pose2d(-36 + 72, -72 + 72);

    private Pose2d spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose2d startPose = new Pose2d(63 + 72, 36, Math.PI);

    // TODO: dont forget to adjust this too
    private Point abortPoint = new Point(135, 120, Point.CARTESIAN);

    private Follower follower;

    private Path scoreSpikeMark, adjustHeadingFromSpikeMark;
    private PathChain initialScoreOnBackdrop, firstCycleToStack, firstCycleStackGrab, firstCycleScoreOnBackdrop, secondCycleToStack, secondCycleStackGrab, secondCycleScoreOnBackdrop;

    private int pathState;

    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose2d(redLeftSideLeftSpikeMark.getX() - 2, redLeftSideLeftSpikeMark.getY() + 0.5, Math.PI / 2);
                initialBackdropGoalPose = new Pose2d(redLeftBackdrop.getX() + 8, redLeftBackdrop.getY() - ROBOT_BACK_LENGTH + 0.25, Math.PI * 1.5);
                break;
            case "middle":
                spikeMarkGoalPose = new Pose2d(redLeftSideMiddleSpikeMark.getX() - 0.4, redLeftSideMiddleSpikeMark.getY() - 4, Math.PI / 2);
                initialBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX() + 10.3, redMiddleBackdrop.getY() - ROBOT_BACK_LENGTH, Math.PI * 1.5);
                break;
            case "right":
                spikeMarkGoalPose = new Pose2d(redLeftSideRightSpikeMark.getX() - 3, redLeftSideRightSpikeMark.getY(), Math.PI / 2);
                initialBackdropGoalPose = new Pose2d(redRightBackdrop.getX() + 7.5, redRightBackdrop.getY() - ROBOT_BACK_LENGTH + 0.25, Math.PI * 1.5);
                break;
        }
    }

    public void buildPaths() {
        Point scoreSpikeMarkMidPoint;
        double scoreSpikeMarkMidToSpikeDistance;
        switch (navigation) {
            default:
            case "left":
                scoreSpikeMarkMidPoint = new Point(startPose.getX() - 12.5, startPose.getY() - 15, Point.CARTESIAN);
                break;
            case "middle":
                scoreSpikeMarkMidPoint = new Point(startPose.getX() - 5.5, startPose.getY() - 22, Point.CARTESIAN);
                break;
            case "right":
                scoreSpikeMarkMidPoint = new Point(startPose.getX() - 3.5, startPose.getY() - 7, Point.CARTESIAN);
                break;
        }
        scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
        scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() + MathFunctions.getSign(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() + MathFunctions.getSign(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
        scoreSpikeMark.setConstantHeadingInterpolation(startPose.getHeading());
        scoreSpikeMark.setPathEndTimeout(3);

        adjustHeadingFromSpikeMark = new Path(new BezierLine(scoreSpikeMark.getLastControlPoint(), new Point(132, 30, Point.CARTESIAN)));
        adjustHeadingFromSpikeMark.setLinearHeadingInterpolation(scoreSpikeMark.getEndTangent().getTheta(), Math.PI * 1.5);

        initialScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(adjustHeadingFromSpikeMark.getLastControlPoint(), new Point(135, 85, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .addPath(new BezierCurve(new Point(135, 85, Point.CARTESIAN), new Point(135, 115, Point.CARTESIAN), new Point(initialBackdropGoalPose)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .setPathEndTimeout(2.5)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10: // starts following the first path to score on the spike mark
                follower.followPath(scoreSpikeMark);
                twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_OUT_POSITION + 0.01);
                setPathState(11);
                break;
            case 11: // detects the path to progress away from the wall and sets tangent interpolation
                if (follower.getCurrentTValue() > 0.1) {
                    //scoreSpikeMark.setReversed(false);
                    scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading() - 0.1 * MathFunctions.getTurnDirection(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()) * MathFunctions.getSmallestAngleDifference(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()), scoreSpikeMark.getEndTangent().getTheta());
                    setPathState(12);
                }
                break;
            case 12: // detects for the end of the path and everything else to be in order and releases the pixel
                if (!follower.isBusy() && twoPersonDrive.intakeState == INTAKE_OUT) {
                    twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_OPEN);
                    setPathState(13);
                }
                break;
            case 13: // moves mechanisms into position to score and pick up from stack as well as starts moving to score
                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTime() > 500) {
                    follower.followPath(adjustHeadingFromSpikeMark);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(16);
                }
                break;
            case 16:
                if (follower.getCurrentPathNumber() == 1) {
                    twoPersonDrive.setTransferState(TRANSFER_OUT);
                    twoPersonDrive.outtakeWristOffset = -15;
                    setPathState(17);
                }
            case 17: // detects for end of the path and outtake out and drops pixel
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    Follower.useHeading = false;
                    follower.holdPoint(new BezierPoint(new Point(follower.getPose())), Math.PI * 1.5);
                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_YELLOW_SCORE_POSITION);
                    setPathState(18);
                }
                break;
            case 18:
                colorSensorDisconnected = colorSensorDisconnected();
                if (colorSensorDisconnected) {
                    setPathState(19);
                    break;
                }
                backdropCorrection();
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(19);
                }
                break;
            case 19:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(110);
                }
                break;
            case 110: // detects for end of the path and outtake out and drops pixel
                if (pathTimer.getElapsedTime() > 500) {
                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
                    setPathState(111);
                }
                break;
            case 111:
                if (pathTimer.getElapsedTime() > 500) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    setPathState(20);
                }
                break;

            case 20:
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    Follower.useHeading = true;
                    twoPersonDrive.moveIntake(INTAKE_IN);
                    setPathState(21);
                }
                break;
            case 21:
                if (twoPersonDrive.intakeState == INTAKE_IN && twoPersonDrive.intakeArmAtTargetPosition() && twoPersonDrive.outtakeState == OUTTAKE_IN && twoPersonDrive.outtakeArmAtTargetPosition() && twoPersonDrive.liftEncoder.getCurrentPosition() < LIFT_TRANSFER_UPPER_LIMIT) {
                    follower.poseUpdater.resetOffset();
                    PathChain abort = follower.pathBuilder()
                            .addPath(new BezierLine(new Point(follower.poseUpdater.getPose()), abortPoint))
                            .setConstantHeadingInterpolation(Math.PI * 1.5)
                            .build();
                    follower.followPath(abort);
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                requestOpModeStop();
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    public boolean backdropCorrection() {
        double rawLightValue = colorSensor.getRawLightDetected();

        if (!(rawLightValue == 0)) {
            // too close
            if (rawLightValue > 150)
                follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() + twoPersonDrive.deltaTimeSeconds * 4);

            // too far
            if (rawLightValue < 134)
                follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() - twoPersonDrive.deltaTimeSeconds * 6);

            if (Math.abs(follower.poseUpdater.getYOffset()) > 1.5)
                follower.poseUpdater.setYOffset(1.5 * MathFunctions.getSign(follower.poseUpdater.getYOffset()));
        } else {
            return false;
        }
        telemetry.addData("raw light value", rawLightValue);

        return true;
    }

    public boolean colorSensorDisconnected() {
        return colorSensor.getRawLightDetected() == 0;
    }

    @Override
    public void loop() {
        follower.update();
        twoPersonDrive.autonomousControlUpdate();

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        double[] motorPowers = follower.motorPowers();
        for (int i = 0; i < motorPowers.length; i++) {
            telemetry.addData("motor " + i, motorPowers[i]);
        }
        twoPersonDrive.telemetry();
        //telemetry.update();
    }

    @Override
    public void init() {
        //PhotonCore.start(this.hardwareMap);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        twoPersonDrive = new TwoPersonDrive(true);
        twoPersonDrive.hardwareMap = hardwareMap;
        twoPersonDrive.telemetry = telemetry;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        scanTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        teamPropPipeline = new VisionPortalTeamPropPipeline(0);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessors(teamPropPipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        twoPersonDrive.initialize();
        twoPersonDrive.setIntakeArmPosition(INTAKE_ARM_IN_POSITION + 0.1);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        if (colorSensorDisconnected()) {
            try {
                throw new Exception("color sensor disconnected");
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_CLOSED);

        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);

        try {
            sleep(2500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        scanTimer.resetTimer();
    }

    @Override
    public void init_loop() {
        if (scanTimer.getElapsedTime() > 750) {
            navigation = teamPropPipeline.getNavigation();
            telemetry.addData("Navigation:", navigation);
            telemetry.update();
            scanTimer.resetTimer();
        } else if (scanTimer.getElapsedTime() > 720) {
            visionPortal.setProcessorEnabled(teamPropPipeline, true);
        } else {
            visionPortal.setProcessorEnabled(teamPropPipeline, false);
        }
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
        setBackdropGoalPose();
        buildPaths();
        twoPersonDrive.frameTimer.resetTimer();
        opmodeTimer.resetTimer();
        setPathState(10);
    }

    @Override
    public void stop() {
    }
}

/**
 * 8==D
 */