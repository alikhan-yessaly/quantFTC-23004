package org.firstinspires.ftc.teamcode.notcompetition.AUTO;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "TestPedroAuto", group = "Autonomous")
public class TestPedroAuto extends OpMode {

    private Timer pathTimer, opmodeTimer, scanTimer;
    private ServoImplEx liftUpServo, liftUpLServo, clawServo, wristServo, arm1Servo, arm0Servo;
    // TODO: adjust this for each auto
    private Pose startPose = new Pose(144-(63+72), 12+72, 0);

    private Follower follower;
    private PathChain firstCycle;

    public void buildPaths() {
        firstCycle = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(7.182, 90.359, Point.CARTESIAN),
                                new Point(33.890, 92.603, Point.CARTESIAN),
                                new Point(16.833, 74.873, Point.CARTESIAN),
                                new Point(39.277, 75.546, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autonomousPathUpdate() {
        if (pathTimer.getElapsedTime() < 500) {
            liftUpServo.setPosition(1);
            liftUpLServo.setPosition(1);
            follower.followPath(firstCycle);
        }
        else if (pathTimer.getElapsedTime() > 3000) {
            liftUpServo.setPosition(1);
            liftUpLServo.setPosition(1  );
        }
        else if (pathTimer.getElapsedTime() > 6000){
            liftUpServo.setPosition(0.7);
            liftUpLServo.setPosition(0.7);
        }
    }

    public void setPathState(int state) {
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("liftUp:", liftUpServo.getPosition());
    }

    @Override
    public void init() {
        // Servo setup
        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
        clawServo = (ServoImplEx)hardwareMap.servo.get("claw");
        wristServo = (ServoImplEx)hardwareMap.servo.get("wrist");
        arm1Servo = (ServoImplEx)hardwareMap.servo.get("arm1");
        arm0Servo = (ServoImplEx)hardwareMap.servo.get("arm0");
        liftUpServo = (ServoImplEx)hardwareMap.get(Servo.class, "liftUp");
        liftUpLServo = (ServoImplEx)hardwareMap.get(Servo.class, "liftUpL");
        liftUpServo.setDirection(Servo.Direction.REVERSE);
        liftUpServo.setPwmRange(pwmRange);
        liftUpLServo.setPwmRange(pwmRange);
        clawServo.setPwmRange(pwmRange);
        wristServo.setPwmRange(pwmRange);
        arm1Servo.setPwmRange(pwmRange);
        arm0Servo.setPwmRange(pwmRange);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        scanTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        scanTimer.resetTimer();
    }

    @Override
    public void start() {
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(10);
    }

    @Override
    public void stop() {
    }
}
