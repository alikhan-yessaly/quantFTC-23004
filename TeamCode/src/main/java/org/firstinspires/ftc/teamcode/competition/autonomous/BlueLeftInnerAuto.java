package org.firstinspires.ftc.teamcode.competition.autonomous;


import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INNER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_AUTO_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_STACK_MIDDLE_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_ARM_STACK_TOP_POSITION;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_CLOSE_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_CLAW_DROP_TIME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.OUTTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ROBOT_FRONT_LENGTH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_OUT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_POSITIONING;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_PRESET_HOLD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.TRANSFER_RESET;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

@Autonomous(name = "Blue Left Inner Auto", group = "Autonomous")
public class BlueLeftInnerAuto extends OpMode {

    private TwoPersonDrive twoPersonDrive;

    private Timer pathTimer, opmodeTimer;

    private VisionPortalTeamPropPipeline teamPropPipeline;

    private VisionPortal visionPortal;

    private String navigation;

    private DistanceSensor leftDistanceSensor, rightDistanceSensor;

    // IMPORTANT: y increasing is towards the backstage from the audience,
    // while x increasing is towards the red side from the blue side
    // this means that 0 heading is pointing from the blue side to the red side

    // all spike mark locations since I'm lazy
    private Pose2d redLeftSideLeftSpikeMark = new Pose2d(36+72,-47.5+72);
    private Pose2d redLeftSideMiddleSpikeMark = new Pose2d(24.5+72,-36+72);
    private Pose2d redLeftSideRightSpikeMark = new Pose2d(36+72,-24.5+72);
    private Pose2d redRightSideLeftSpikeMark = new Pose2d(36+72, 0.5+72);
    private Pose2d redRightSideMiddleSpikeMark = new Pose2d(24.5+72, 12+72);
    private Pose2d redRightSideRightSpikeMark = new Pose2d(36+72, 23.5+72);
    private Pose2d blueLeftSideLeftSpikeMark = new Pose2d(-36+72, 23.5+72);
    private Pose2d blueLeftSideMiddleSpikeMark = new Pose2d(-24.5+72, 12+72);
    private Pose2d blueLeftSideRightSpikeMark = new Pose2d(-36+72, 0.5+72);
    private Pose2d blueRightSideLeftSpikeMark = new Pose2d(-36+72, -24.5+72);
    private Pose2d blueRightSideMiddleSpikeMark = new Pose2d(-24.5+72, -36+72);
    private Pose2d blueRightSideRightSpikeMark = new Pose2d(-36+72, -47.5+72);

    // backdrop april tag locations
    private Pose2d blueLeftBackdrop = new Pose2d(-42.875+72, 60.75+72);
    private Pose2d blueMiddleBackdrop = new Pose2d(-36.75+72, 60.75+72);
    private Pose2d blueRightBackdrop = new Pose2d(-30.75+72, 60.75+72);
    private Pose2d redLeftBackdrop = new Pose2d(30.75+72, 60.75+72);
    private Pose2d redMiddleBackdrop = new Pose2d(36.75+72, 60.75+72);
    private Pose2d redRightBackdrop = new Pose2d(42.875+72, 60.75+72);

    // white pixel stack locations
    private Pose2d redOuterStack = new Pose2d(36+72, -72+72);
    private Pose2d redMiddleStack = new Pose2d(24+72, -72+72);
    private Pose2d redInnerStack = new Pose2d(12+72,-72+72);
    private Pose2d blueInnerStack = new Pose2d(-12+72,-72+72);
    private Pose2d blueMiddleStack = new Pose2d(-24+72, -72+72);
    private Pose2d blueOuterStack = new Pose2d(-36+72, -72+72);

    private Pose2d spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose, thirdCycleStackPose, thirdCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose2d startPose = new Pose2d(144-(63+72), 12+72, 0);

    private Follower follower;

    private Path scoreSpikeMark, initialScoreOnBackdrop;
    private PathChain firstCycleToStack, firstCycleStackGrab, firstCycleScoreOnBackdrop, secondCycleToStack, secondCycleStackGrab, secondCycleScoreOnBackdrop;

    private int pathState;

    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose2d(blueLeftSideLeftSpikeMark.getX() + 2.5, blueLeftSideLeftSpikeMark.getY()+1, Math.PI/2);
                initialBackdropGoalPose = new Pose2d(blueLeftBackdrop.getX(), blueLeftBackdrop.getY()-ROBOT_BACK_LENGTH-0.5, Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX(), blueRightBackdrop.getY()-ROBOT_BACK_LENGTH+0.25, Math.PI * 1.5);
                secondCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX(), blueRightBackdrop.getY()-ROBOT_BACK_LENGTH+0.25, Math.PI * 1.5);
                break;
            case "middle":
                spikeMarkGoalPose = new Pose2d(blueLeftSideMiddleSpikeMark.getX(), blueLeftSideMiddleSpikeMark.getY()+4, Math.PI/2);
                initialBackdropGoalPose = new Pose2d(blueMiddleBackdrop.getX() + 0.5, blueMiddleBackdrop.getY()-ROBOT_BACK_LENGTH-0.5,Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX(), blueRightBackdrop.getY()-ROBOT_BACK_LENGTH+0.25, Math.PI * 1.5);
                secondCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX(), blueRightBackdrop.getY()-ROBOT_BACK_LENGTH+0.25, Math.PI * 1.5);
                break;
            case "right":
                spikeMarkGoalPose = new Pose2d(blueLeftSideRightSpikeMark.getX() + 2.5, blueLeftSideRightSpikeMark.getY()-1.5, Math.PI/2);
                initialBackdropGoalPose = new Pose2d(blueRightBackdrop.getX(), blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-0.5, Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX(), blueRightBackdrop.getY()-ROBOT_BACK_LENGTH+0.25, Math.PI * 1.5);
                secondCycleBackdropGoalPose = new Pose2d(blueRightBackdrop.getX(), blueRightBackdrop.getY()-ROBOT_BACK_LENGTH+0.25, Math.PI * 1.5);
                break;
        }
    }

    public void buildPaths() {
        Point scoreSpikeMarkMidPoint;
        double scoreSpikeMarkMidToSpikeDistance;
        switch (navigation) {
            default:
            case "left":
                scoreSpikeMarkMidPoint = new Point(144-131.5, 82, Point.CARTESIAN);
                scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
                scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() - Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() - Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
                break;
            case "middle":
                scoreSpikeMarkMidPoint = new Point(144-129.5, 106, Point.CARTESIAN);
                scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
                scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() - Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() + Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
                break;
            case "right":
                scoreSpikeMarkMidPoint = new Point(144-122.5, 99, Point.CARTESIAN);
                scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
                scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() - Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() + Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
                break;
        }
        scoreSpikeMark.setPathEndTimeout(2);

        switch (navigation) {
            default:
            case "left":
                initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(scoreSpikeMark.getLastControlPoint().getX(), 98, Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 109.5, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
                break;
            case "middle":
                initialScoreOnBackdrop = new Path(new BezierLine(scoreSpikeMark.getLastControlPoint(), new Point(initialBackdropGoalPose)));
                break;
            case "right":
                initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(144-135, 98, Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 106, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
                break;
        }
        initialScoreOnBackdrop.setConstantHeadingInterpolation(Math.PI * 1.5);
        initialScoreOnBackdrop.setPathEndTimeout(2.5);

        switch (navigation) {
            default:
            case "left":
                firstCycleStackPose = new Pose2d(blueInnerStack.getX()-0.5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH, Math.PI * 1.5 + Math.toRadians(1));
                secondCycleStackPose = new Pose2d(blueInnerStack.getX()-0.5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH, Math.PI * 1.5 + Math.toRadians(1));
                break;
            case "middle":
                firstCycleStackPose = new Pose2d(blueInnerStack.getX()-0.5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH, Math.PI * 1.5 + Math.toRadians(1));
                secondCycleStackPose = new Pose2d(blueInnerStack.getX()-1, blueInnerStack.getY() + ROBOT_FRONT_LENGTH, Math.PI * 1.5 + Math.toRadians(1));
                break;
            case "right":
                firstCycleStackPose = new Pose2d(blueInnerStack.getX()-0.5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH, Math.PI * 1.5 + Math.toRadians(1));
                secondCycleStackPose = new Pose2d(blueInnerStack.getX()-1, blueInnerStack.getY() + ROBOT_FRONT_LENGTH, Math.PI * 1.5 + Math.toRadians(1));
                break;
        }

        firstCycleToStack = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initialBackdropGoalPose), new Point(144-76.5, 106, Point.CARTESIAN), new Point(144-80, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .addPath(new BezierLine(new Point(144-84, 79, Point.CARTESIAN), new Point(firstCycleStackPose.getX(), 28, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .setPathEndTimeout(0)
                .build();

        firstCycleStackGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose.getX(), 32, Point.CARTESIAN), new Point(firstCycleStackPose)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .build();

        firstCycleScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose), new Point(144-84, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .addPath(new BezierCurve(new Point(144-84, 79, Point.CARTESIAN), new Point(144-76.5, 106-24, Point.CARTESIAN), new Point(firstCycleBackdropGoalPose)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .setPathEndTimeout(2.5)
                .build();

        secondCycleToStack = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstCycleBackdropGoalPose), new Point(144-76.5, 106, Point.CARTESIAN), new Point(144-84, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .addPath(new BezierLine(new Point(144-84, 79, Point.CARTESIAN), new Point(secondCycleStackPose.getX(), 28, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .setPathEndTimeout(0)
                .build();

        secondCycleStackGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondCycleStackPose.getX(), 32, Point.CARTESIAN), new Point(secondCycleStackPose)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .build();

        secondCycleScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondCycleStackPose), new Point(144-84, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .addPath(new BezierCurve(new Point(144-84, 79, Point.CARTESIAN), new Point(144-76.5, 106-24, Point.CARTESIAN), new Point(secondCycleBackdropGoalPose)))
                .setPathEndTimeout(2.5)
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10: // starts following the first path to score on the spike mark
                follower.followPath(scoreSpikeMark);
                twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_OUT_POSITION+0.01);
                setPathState(11);
                break;
            case 11: // detects the path to progress away from the wall and sets tangent interpolation
                if (follower.getCurrentTValue() > 0.1) {
                    scoreSpikeMark.setReversed(false);
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
                    twoPersonDrive.setTransferState(TRANSFER_OUT);
                    twoPersonDrive.outtakeWristOffset = -15;
                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTime() > 500) {
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(15);
                }
                break;
            case 15: // detects for end of the path and outtake out and drops pixel
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    Follower.useHeading = false;
                    follower.holdPoint(new BezierPoint(new Point(follower.getPose())), Math.PI * 1.5);
                    twoPersonDrive.setOuttakeArmInterpolation(0.36);
                    setPathState(16);
                }
                break;
            case 16:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(17);
                }
                break;
            case 17: // detects for end of the path and outtake out and drops pixel
                if (pathTimer.getElapsedTime() > 500) {
                    twoPersonDrive.innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_OPEN);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTime() > 500) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    setPathState(20);
                }
                break;


            case 20: // starts the robot off on to the first stack once the pixels have been dropped
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    Follower.useHeading = true;
                    follower.followPath(firstCycleToStack);
                    setPathState(21);
                }
                break;
            case 21:
                if (follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.4 && follower.isBusy()) {
                    stackCorrection();
                }
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(firstCycleToStack.getPath(1).getLastControlPoint().getX(),firstCycleToStack.getPath(1).getLastControlPoint().getY() + 1, Point.CARTESIAN)), firstCycleStackPose.getHeading());
                    setPathState(22);
                }
                break;
            case 22:
                twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_TOP_POSITION);
                setPathState(23);
                break;
            case 23:
                if (twoPersonDrive.intakeArmAtTargetPosition()) {
                    setPathState(24);
                }
                break;
            case 24:
                if (pathTimer.getElapsedTime() > 500) {
                    follower.followPath(firstCycleStackGrab);
                    setPathState(25);
                }
                break;
            case 25:
                if (follower.getCurrentTValue() > 0.92) {//!follower.isBusy()) {
                    //Follower.useHeading = false;
                    //follower.holdPoint(new BezierPoint(new Point(firstCycleStackPose)), Math.PI * 1.5);
                    twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);
                    setPathState(26);
                }
                if (pathTimer.getElapsedTime() > 3000) {
                    setPathState(26);
                }
                break;
            case 26: // waits for the intake claw to close and then sets the intake to move back in while pulling the extension back in slightly
                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_POSITIONING);
                    follower.poseUpdater.resetOffset();
                    Follower.useHeading = true;
                    follower.followPath(firstCycleScoreOnBackdrop);
                    setPathState(27);
                }
                break;
            case 27:
                if (((follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.1) || !follower.isBusy()) && twoPersonDrive.transferState == TRANSFER_PRESET_HOLD) {
                    twoPersonDrive.liftPresetTargetPosition = 850;
                    twoPersonDrive.setTransferState(TRANSFER_OUT);
                    setPathState(28);
                }
                break;
            case 28: // detects for end of the path and outtake out and drops pixel
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    twoPersonDrive.setOuttakeArmInterpolation(0.4, 100);
                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
                    //Follower.useHeading = false;
                    follower.holdPoint(new BezierPoint(new Point(follower.getPose())), Math.PI * 1.5);
                    setPathState(29);
                }
                break;
            case 29:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(210);
                }
                break;
            case 210:
                if (pathTimer.getElapsedTime() > 300) {
                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
                    setPathState(211);
                }
                break;
            case 211: // once the outer pixel has dropped, drop the inner one and fold up
                if (pathTimer.getElapsedTime() > 2*OUTTAKE_CLAW_DROP_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    setPathState(30);
                }
                break;


            case 30: // once the inner pixel has dropped, start the robot off to the second pass on the first stack
                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
                    Follower.useHeading = true;
                    follower.followPath(secondCycleToStack);
                    setPathState(31);
                }
                break;
            case 31:
                if (follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.4 && follower.isBusy()) {
                    stackCorrection();
                }
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(new Point(secondCycleToStack.getPath(1).getLastControlPoint().getX(),secondCycleToStack.getPath(1).getLastControlPoint().getY() + 1, Point.CARTESIAN)), secondCycleStackPose.getHeading());
                    setPathState(32);
                }
                break;
            case 32:
                twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_MIDDLE_POSITION);
                setPathState(33);
                break;
            case 33:
                if (twoPersonDrive.intakeArmAtTargetPosition()) {
                    setPathState(34);
                }
                break;
            case 34:
                if (pathTimer.getElapsedTime() > 300) {
                    follower.followPath(secondCycleStackGrab);
                    setPathState(35);
                }
                break;
            case 35:
                if (follower.getCurrentTValue() > 0.92) {//!follower.isBusy()) {
                    //Follower.useHeading = false;
                    //follower.holdPoint(new BezierPoint(new Point(secondCycleStackPose)), Math.PI * 1.5);
                    twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);
                    setPathState(36);
                }
                if (pathTimer.getElapsedTime() > 3000) {
                    setPathState(36);
                }
                break;
            case 36: // waits for the intake claw to close and then sets the intake to move back in while pulling the extension back in slightly
                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_POSITIONING);
                    //Follower.useHeading = true;
                    follower.poseUpdater.resetOffset();
                    follower.followPath(secondCycleScoreOnBackdrop);
                    setPathState(37);
                }
                break;
            case 37:
                if (((follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.1) || !follower.isBusy()) && twoPersonDrive.transferState == TRANSFER_PRESET_HOLD) {
                    twoPersonDrive.liftPresetTargetPosition = 850;
                    twoPersonDrive.setTransferState(TRANSFER_OUT);
                    setPathState(38);
                }
                break;
            case 38: // detects for end of the path and outtake out and drops pixel
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    twoPersonDrive.setOuttakeArmInterpolation(0.4, 100);
                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
                    //Follower.useHeading = false;
                    follower.holdPoint(new BezierPoint(new Point(follower.getPose())), Math.PI * 1.5);
                    setPathState(39);
                }
                break;
            case 39:
                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
                    setPathState(310);
                }
                break;
            case 310:
                if (pathTimer.getElapsedTime() > 300) {
                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
                    setPathState(311);
                }
                break;
            case 311: // once the outer pixel has dropped, drop the inner one and fold up
                if (pathTimer.getElapsedTime() > 2*OUTTAKE_CLAW_DROP_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_RESET);
                    Follower.useHeading = true;
                    setPathState(40);
                }
                break;


            case 40: // move the intake in
                twoPersonDrive.moveIntake(INTAKE_IN);
                setPathState(41);
                break;
            case 41: // once the robot is nice and folded up, request stop
                if (twoPersonDrive.intakeState == INTAKE_IN && twoPersonDrive.intakeArmAtTargetPosition() && twoPersonDrive.outtakeState == OUTTAKE_IN && twoPersonDrive.outtakeArmAtTargetPosition()) {
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

    public void stackCorrection() {
        double error = leftDistanceSensor.getDistance(DistanceUnit.INCH)-rightDistanceSensor.getDistance(DistanceUnit.INCH);

        if (Math.abs(error) > 0.75) follower.poseUpdater.setXOffset(follower.poseUpdater.getXOffset() + twoPersonDrive.deltaTimeSeconds * 5 * MathFunctions.getSign(error));

        if (Math.abs(follower.poseUpdater.getXOffset()) > 2) follower.poseUpdater.setXOffset(2 * MathFunctions.getSign(follower.poseUpdater.getXOffset()));

        telemetry.addData("error", error);
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

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        twoPersonDrive = new TwoPersonDrive(true);
        twoPersonDrive.hardwareMap = hardwareMap;
        twoPersonDrive.telemetry = telemetry;

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        teamPropPipeline = new VisionPortalTeamPropPipeline(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessors(teamPropPipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        twoPersonDrive.initialize();

        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);

        twoPersonDrive.innerOuttakeClaw.setPosition(INNER_OUTTAKE_CLAW_CLOSED);
    }

    @Override
    public void init_loop() {
        super.init_loop();

        navigation = teamPropPipeline.getNavigation();
        telemetry.addData("Navigation:", navigation);
        telemetry.update();
        setBackdropGoalPose();
        buildPaths();
    }

    @Override
    public void start() {
        super.start();
        visionPortal.stopStreaming();
        twoPersonDrive.frameTimer.resetTimer();
        opmodeTimer.resetTimer();
        setPathState(10);
    }

    @Override
    public void stop() {
        super.stop();
    }
}

/**
 * 8==D
 */